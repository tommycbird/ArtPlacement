#include <fstream>
#include <list>
#include <cassert>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polygon_with_holes_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>

#include <boost/property_map/property_map.hpp>

#include "pgn_print.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Face_handle                                          Face_handle;
typedef CDT::Point                                                Point;
typedef K::Point_3                                                Point_3;
typedef CGAL::Surface_mesh<Point_3>                               Mesh;
typedef K::Point_2                                                Point_2;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>                             Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                           Pgn_with_holes_2_container;


bool isVisible(Face_handle f, Polygon_with_holes_2 p, Point_2 person) {
  // Check each vertice of the triangle
  for(int i = 0; i < 3; i++) {
    //Check if a line between the person and the vertice intersects with the polygon, if not, return true

    return true;
  }
  return false;
}

int main(int argc, char* argv[])
{
  // ./ArtPlacement input_room.txt 0 200 2"
  // ./ArtPlacement input_class.txt 1 250 2"
  if(argc != 5) { 
    std::cout << "USAGE: ./ArtPlacement filename.txt noise_type accuracy precision" << std::endl;
    std::cout << "noise_type: 0 = white, 1 = blue" << std::endl;
  }

  // Open the input file and read the two polygons from it.
  const char* filename = "../rooms_star.dat";
  std::ifstream    in_file(filename);
  if (! in_file.is_open()) {
    std::cerr << "Failed to open the input file." << std::endl;
    return -1;
  }
  Polygon_2 floor_plan, Q;
  in_file >> floor_plan >> Q;
  in_file.close();

  // Parse the command line arguments.
  int precision = 2;
  int accuracy = 200;
  bool blue_noise = true; // true = blue, false = white

  // Draw the two polygons.
  //CGAL::draw(floor_plan);
  //CGAL::draw(Q);

  // Compute and print the Minkowski sum.
  auto sum = CGAL::minkowski_sum_2(floor_plan, Q);
  print_polygon_with_holes(sum);
  //CGAL::draw(sum);


  // Create a list of points and store the outer boundary of the minkowski sum
  std::list<Point_2> points;
  for(auto it = sum.outer_boundary().vertices_begin(); it != sum.outer_boundary().vertices_end(); ++it) {
    points.push_back(*it);
  }

  // Add midpoint of each edge in sum to points
  if(precision == 1) {
    for(auto it = sum.outer_boundary().edges_begin(); it != sum.outer_boundary().edges_end(); ++it) {
      points.push_back(CGAL::midpoint(it->source(), it->target()));
    }
  }else if(precision > 1) {
    for(auto it = sum.outer_boundary().edges_begin(); it != sum.outer_boundary().edges_end(); ++it) {
      Point_2 mid2 = CGAL::midpoint(it->source(), it->target());
      Point_2 mid1 = CGAL::midpoint(it->source(), mid2);
      Point_2 mid3 = CGAL::midpoint(mid2, it->target());
      points.push_back(mid1);
      points.push_back(mid2);
      points.push_back(mid3);
    }
  }

  // ADD CENTROIDS ========== [ DEPRECATED METHOD ] ==========
  // for(auto it = sum.holes_begin(); it != sum.holes_end(); ++it) {
  //   for(auto it2 = it->vertices_begin(); it2 != it->vertices_end(); ++it2) {
  //     points.push_back(*it2);
  //   }
  // }
  // for(int i = 0; i < precision; i++){
  //   // Create a triangulation
  //   CDT cdt_temp;
  //   // Insert the points into the triangulation
  //   cdt_temp.insert_constraint(points.begin(), points.end());
  //   //Mark facets that are inside the domain bounded by the polygon
  //   CGAL::draw(cdt_temp);
  //   // Loop through all triangles
  //   for(Face_handle f : cdt_temp.finite_face_handles()) {
  //     // Only marked faces
  //     // Get the vertices of the triangle
  //     Point_2 p1 = f->vertex(0)->point();
  //     Point_2 p2 = f->vertex(1)->point();
  //     Point_2 p3 = f->vertex(2)->point();
  //     // add centroid to points
  //     Point_2 p4 = CGAL::centroid(p1, p2, p3);
  //     points.push_back(p4);
  //   }
  // }


  // Triangulate the minkowski sum
  CDT cdt;
  //cdt.insert_constraint(outer.vertices_begin(), outer.vertices_end(), true);
  cdt.insert_constraint(points.begin(), points.end(), true);
  std::unordered_map<Face_handle, bool> in_domain_map;
  boost::associative_property_map< std::unordered_map<Face_handle,bool> >
    in_domain(in_domain_map);
  //Mark facets that are inside the domain bounded by the polygon
  CGAL::mark_domain_in_triangulation(cdt, in_domain);
  CGAL::draw(cdt, in_domain);

  // ADD "TRIFORCES" ========== [ DEPRECATED METHOD ] ==========
  // // Turn every triangle into more triangles
  // for(int i = 0; i < precision; i++){
  //   // Loop through all triangles
  //   for(Face_handle f : cdt.finite_face_handles()) {
  //     // Skip unmarked faces
  //     if(!in_domain_map[f]) {
  //       continue;
  //     }
  //     // Get the vertices of the triangle
  //     Point_2 p1 = f->vertex(0)->point();
  //     Point_2 p2 = f->vertex(1)->point();
  //     Point_2 p3 = f->vertex(2)->point();
  //     // create new face with midpoint of each edge
  //     Point_2 p4 = CGAL::midpoint(p1, p2);
  //     Point_2 p5 = CGAL::midpoint(p2, p3);
  //     Point_2 p6 = CGAL::midpoint(p3, p1);
  //     // Make new polygon from the 3 new points
  //     Polygon_2 smaller_tri;
  //     smaller_tri.push_back(p4);
  //     smaller_tri.push_back(p5);
  //     smaller_tri.push_back(p6);
  //     // Mark the new face as inside the domain
  //     cdt.insert_constraint(smaller_tri.vertices_begin(), smaller_tri.vertices_end(), true);
  //   }
  // }
  // CGAL::draw(cdt, in_domain);


  // Create surface mesh from triangulation
  Mesh heatmap;
  // Add marked faces to surface mesh
  for(Face_handle f : cdt.finite_face_handles()) {
    if(in_domain_map[f]) {
      Point_2 p1 = f->vertex(0)->point();
      Point_2 p2 = f->vertex(1)->point();
      Point_2 p3 = f->vertex(2)->point();
      Mesh::Vertex_index u = heatmap.add_vertex(Point_3(p1.x(), p1.y(), 0.0));
      Mesh::Vertex_index v = heatmap.add_vertex(Point_3(p2.x(), p2.y(), 0.0));
      Mesh::Vertex_index w = heatmap.add_vertex(Point_3(p3.x(), p3.y(), 0.0));
      heatmap.add_face(u, v, w);
    }
  }
  CGAL::draw(heatmap);

  // Get the min and max x and y values
  float min_x = 2000.0;
  float max_x = 0.0;
  float min_y = 2000.0;
  float max_y = 0.0;
  for(auto it = sum.outer_boundary().vertices_begin(); it != sum.outer_boundary().vertices_end(); ++it) {
    if(it->x() < min_x) {
      min_x = it->x();
    }
    if(it->x() > max_x) {
      max_x = it->x();
    }
    if(it->y() < min_y) {
      min_y = it->y();
    }
    if(it->y() > max_y) {
      max_y = it->y();
    }
  }

  // Generate accuracy random points (people) in a circle from center with radius max_dist, depending on noise_type
  std::vector<Point_2> people;
  // Go through each triangle
  while(people.size() < accuracy){
    // Generate a random point between min and max x and y
    float x = min_x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x-min_x)));
    float y = min_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_y-min_y)));
    Point_2 person(x, y);
    auto os = CGAL::oriented_side(person, floor_plan);

    // Add the point if its not inside the floor_plan polygon
    if(os != CGAL::POSITIVE) {
      if(blue_noise) {
        // Check if the point is too close to another point
        bool too_close = false;
        for(int i = 0; i < people.size(); i++) {
          if(CGAL::squared_distance(person, people[i]) < 0.1) {
            too_close = true;
            break;
          }
        }
        if(too_close) {
          continue;
        }
      }
      people.push_back(person);
    }
  }

  std::cout << "Number of people: " << people.size() << std::endl;

  //Add people to surface mesh and draw
  Polygon_2 dots;
  for(int i = 0; i < people.size(); i++) {
    dots.push_back(people[i]);
  }
  CGAL::draw(dots);


  // Compute visible triangles (no intersection on input graph edges)
  std::map<Face_handle, float> tri_vis;
  float max_vis = 0.0;
  for(int i = 0; i < people.size(); i++) {
    //loop through all triangles, check if point can see triangle
    for(Face_handle f : cdt.finite_face_handles()) {
      //check if point is inside triangle
      if(isVisible(f, sum, people[i])) {
        tri_vis[f] += 1.0;
        if(tri_vis[f] > max_vis) {
          max_vis = tri_vis[f];
        }
      }
    }
  }

  // Color each triangle according to tri_vis, 0 - max_vis (red - green)

  return 0;
}