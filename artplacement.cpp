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
#include <CGAL/Segment_2.h>

#include <boost/property_map/property_map.hpp>

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
typedef Mesh::face_index                                          face_desc;
typedef K::Point_2                                                Point_2;
typedef CGAL::Polygon_2<K>                                        Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>                             Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                           Pgn_with_holes_2_container;


// Takes in a list of points, a polygon, and a person
// Returns true if the person has line of site to at least one point without intersecting the polygon
bool isVisible(std::vector<Point_2> points, Polygon_2 &fp, Point_2 person) {
  // Loop through all points
  int count = 0;
  for(int i = 0; i < points.size(); i++) {
    // Check if the line between point[i] and person intersects any of fp's edges
    CGAL::Segment_2<K> line(points[i], person);
    bool intersects = false;
    for(auto it = fp.edges_begin(); it != fp.edges_end(); ++it){
      if(CGAL::do_intersect(line, *it)) {
        // Line of site does not exist
        intersects = true;
        break;
      }
    }
    if(!intersects) {
      count++;
      if(count >= 2) return true;
    }
  }
  
  return false;
}

int main(int argc, char* argv[])
{
  // ./ArtPlacement ../input_room.dat 0 200 2"
  // ./ArtPlacement ../input_class.dat 1 250 2"
  if(argc != 5) { 
    std::cout << "USAGE: ./ArtPlacement filename.dat noise_type accuracy precision" << std::endl;
    std::cout << "noise_type: 0 = white, 1 = blue" << std::endl;
    return 1;
  }

  // Read in input file and box file (box is for minkowski sum)
  const char* filename = argv[1];
  const char* box_filename = "../box_input.dat";
  std::ifstream    in_file(filename);
  if (! in_file.is_open()) {
    std::cerr << "Failed to open the input file." << std::endl;
    return -1;
  }
  std::ifstream    box_file(box_filename);
  if (! box_file.is_open()) {
    std::cerr << "Failed to open the box file." << std::endl;
    return -1;
  }
  Polygon_2 floor_plan, box;
  in_file >> floor_plan;
  box_file >> box;
  in_file.close();
  box_file.close();

  // Read arguments to determine parameters
  bool blue_noise = true;
  float epsilon = 0.1; // how close points can be to each other for blue noise
  if(std::stoi(argv[2]) !=1) {
    blue_noise = false;
  }
  int precision = std::stoi(argv[4]);
  int accuracy = std::stoi(argv[3]);
  std:: cout << "Running with noise_type: " << (blue_noise ? "blue" : "white") << ", accuracy: " << argv[3] << ", precision: " << argv[4] << std::endl;

  // Compute the Minkowski sum of the floor plan
  auto sum = CGAL::minkowski_sum_2(floor_plan, box);

  // Create a list of points, storing outer boundary of the minkowski sum
  std::list<Point_2> points;
  for(auto it = sum.outer_boundary().vertices_begin(); it != sum.outer_boundary().vertices_end(); ++it) {
    points.push_back(*it);
  }

  // Add midpoint of each edge in sum to points for each level of precision
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


  // Triangulate the minkowski sum
  CDT cdt;
  //cdt.insert_constraint(outer.vertices_begin(), outer.vertices_end(), true);
  cdt.insert_constraint(points.begin(), points.end(), true);
  std::unordered_map<Face_handle, bool> in_domain_map;
  boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);
  //Mark facets that are inside the domain bounded by the polygon
  CGAL::mark_domain_in_triangulation(cdt, in_domain);
  
  
  // Create surface mesh from the triangulation for coloring
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

  // Get the min and max x and y values from the surface mesh
  float min_x = 2000.0;
  float max_x = 0.0;
  float min_y = 2000.0;
  float max_y = 0.0;
  for(auto it = sum.outer_boundary().vertices_begin(); it != sum.outer_boundary().vertices_end(); ++it) {
    if(it->x() < min_x) min_x = it->x();
    if(it->x() > max_x) max_x = it->x();
    if(it->y() < min_y) min_y = it->y();
    if(it->y() > max_y) max_y = it->y();
  }

  // Generate randomly placed people according to noise pattern
  std::vector<Point_2> people;
  while(people.size() < accuracy){
    // Generate a random point between min and max x and y
    float x = min_x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x-min_x)));
    float y = min_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_y-min_y)));
    Point_2 person(x, y);
    auto os = CGAL::oriented_side(person, floor_plan);
    // Add the point if its not inside the floor_plan polygon
    if(os != CGAL::POSITIVE) {
      if(blue_noise) {
        // For blue noise, check that the point is not too close to another point
        bool too_close = false;
        for(int i = 0; i < people.size(); i++) {
          if(CGAL::squared_distance(person, people[i]) < epsilon) {
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

  //Add people to surface mesh and draw them to see people placement
  /*Polygon_2 dots;
  for(int i = 0; i < people.size(); i++) {
    dots.push_back(people[i]);
  }
  CGAL::draw(dots);*/

  // Compute visible triangles (no intersection on input graph edges)
  std::map<face_desc, float> tri_vis;
  float min_vis = 0.0;
  float max_vis = 0.0;
  float avg_vis = 0.0;
  for(int i = 0; i < people.size(); i++) {
    // Loop through heatmap, check if point can see triangle
    for(auto f : heatmap.faces()) {
      // Get the vertices of the surface mesh face
      std::vector<Point_2> verts;
      for(auto h : heatmap.halfedges_around_face(heatmap.halfedge(f))) {
          Point_3 p3 = heatmap.point(heatmap.target(h));
          Point_2 p2(p3.x(), p3.y());
          verts.push_back(p2);
      }
      // Check if you can draw a line between this person and the triangle without crossing the floor_plan edges
      if(verts.size() == 3 && isVisible(verts, floor_plan, people[i])) {
        tri_vis[f] += 1.0;
        if(tri_vis[f] > max_vis) {
          max_vis = tri_vis[f];
        }
      }
    }
  }
  min_vis = max_vis;
  // Compute average visibility
  for(auto it = tri_vis.begin(); it != tri_vis.end(); ++it) {
    avg_vis += it->second;
    if(it->second < min_vis) {
      min_vis = it->second;
    }
  }
  avg_vis /= tri_vis.size();

  std::cout << "Min visibility: " << min_vis << std::endl;
  std::cout << "Max visibility: " << max_vis << std::endl;
  std::cout << "Average visibility: " << avg_vis << std::endl;

  // Color each face according to tri_vis, 0 - max_vis (red - green)
  auto fcm = heatmap.add_property_map<Mesh::Face_index>("f:color", CGAL::IO::white()).first;
  for(auto f : heatmap.faces()) {
    float vis = tri_vis[f];
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
    if(vis > 0.0) {
      r = 255.0 - (255.0 * (vis / max_vis));
      g = 255.0 * (vis / max_vis);
    }
    put(fcm, f, CGAL::Color(r, g, b));
  }

  CGAL_USE(fcm);
  CGAL::draw(heatmap);

  return 0;
}