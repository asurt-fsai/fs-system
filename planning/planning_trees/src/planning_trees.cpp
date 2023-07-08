#include <array>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../include/tree_search.h"

typedef std::vector<std::array<float, 2>> path_arr;

path_arr toPathArr(Path path){
  std::vector<std::array<float, 2>> path_waypoints;
  for (int i = 0; i < path.waypoints.size(); i++) {
    path_waypoints.push_back({path.waypoints[i].x, path.waypoints[i].y});
  }
  return path_waypoints;
}

std::tuple<path_arr, std::vector<path_arr>, std::vector<Point>, std::vector<float>, std::vector<std::array<float, 3>>, std::vector<std::vector<float>>, std::vector<std::vector<float>>> plan_path(std::vector<float> cone_x,
                                            std::vector<float> cone_y,
                                            std::vector<int> cone_type) {

  std::vector<Cone> cones;
  TreeSearchParams params;
  for (int i = 0; i < cone_x.size(); i++) {
    cones.emplace_back(cone_x[i], cone_y[i], cone_type[i]);
  }
  TreeSearch tree_search = TreeSearch(cones, params);
  std::tuple<Path, std::vector<Path>> output = tree_search.getPath();

  path_arr output_path = toPathArr(std::get<0>(output));
  std::vector<path_arr> explored_paths;
  std::vector<float> costs;
  std::vector<std::array<float, 3>> detailed_costs;
  std::vector<std::vector<float>> all_left_dists;
  std::vector<std::vector<float>> all_right_dists;
  for (int i = 0; i < std::get<1>(output).size(); i++)
  {
    explored_paths.push_back(toPathArr(std::get<1>(output)[i]));
    costs.push_back(std::get<1>(output)[i].getCost());
    std::array<float, 3> detailed_cost;
    detailed_cost[0] = std::get<1>(output)[i].getPathWidthCost();
    detailed_cost[1] = std::get<1>(output)[i].getColorCost();
    detailed_cost[2] = std::get<1>(output)[i].getAngleCost();
    detailed_costs.push_back(detailed_cost);
    all_left_dists.push_back(std::get<1>(output)[i].left_dists);
    all_right_dists.push_back(std::get<1>(output)[i].right_dists);
  }
    return std::tuple<path_arr, std::vector<path_arr>, std::vector<Point>, std::vector<float>, std::vector<std::array<float, 3>>, std::vector<std::vector<float>>, std::vector<std::vector<float>>>(output_path, explored_paths, tree_search.waypoints, costs, detailed_costs, all_left_dists, all_right_dists);
}

PYBIND11_MODULE(py_planning_trees, m) {
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
