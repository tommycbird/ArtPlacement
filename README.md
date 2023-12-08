# Art Placement

This program takes in an input of a floorplan, and outputs a heatmap specifying where it is best to place art based on visibility. It uses the CGAL library to generate a Minkowski sum of the floorplan, then triangulate it using a constrained Delaunay triangulation algorithm. It then uses a raycasting algorithm to determine point visibility, with options for both white noise and blue noise placement.

## Use instructions

You can download and run this program with CGAL. When the results are drawn, press M then E to enable multi-color mode and hide edges for optimal results.
