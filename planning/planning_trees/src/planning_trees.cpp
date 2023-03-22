#include <array>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../include/tree_search.h"

std::vector<std::array<float, 2>> plan_path(std::vector<float> cone_x,
                                            std::vector<float> cone_y,
                                            std::vector<int> cone_type) {
  std::vector<Cone> cones;
  TreeSearchParams params;
  for (int i = 0; i < cone_x.size(); i++) {
    cones.emplace_back(cone_x[i], cone_y[i], cone_type[i]);
  }
  TreeSearch tree_search = TreeSearch(cones, params);
  Path path = tree_search.getPath();
  std::vector<std::array<float, 2>> path_waypoints;
  for (int i = 0; i < path.waypoints.size(); i++) {
    path_waypoints.push_back({path.waypoints[i].x, path.waypoints[i].y});
  }
  return path_waypoints;
}

PYBIND11_MODULE(planning_trees, m) {
  m.doc() = "pybind11 planning_trees plugin";

  m.def(
      "plan_path", &plan_path,
      "A function to find the optimal path between cones given their locations\n\
    \n\
    Parameters\n\
    ----------\n\
    cone_x : List[float]\n\
        A list of the x coordinates of the cones\n\
    cone_y : List[float]\n\
        A list of the y coordinates of the cones\n\
    cone_type : List[int]\n\
        A list of the type of cones\n\
    \n\
    Returns\n\
    -------\n\
    List[List[float]]\n\
        A list of the x and y coordinates of the path");
}
