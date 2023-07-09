#include "../include/path_queue.h"

bool PathQueue::addNewPath(Path new_path, float new_cost) {
  if (paths.size() >= PATH_QUEUE_LIMIT) {
    int worst_path_index = getWorstPathIndex();
    if (costs[worst_path_index] > new_cost) {
      paths[worst_path_index] = new_path;
      costs[worst_path_index] = new_cost;
      return true;
    }
    return false;
  }

  paths.emplace_back(new_path.cones, new_path.cost_params);
  paths.back() = new_path;
  costs.push_back(new_cost);
  return true;
}

int PathQueue::getWorstPathIndex() {
  int worst_path_index = 0;
  for (int i = 0; i < costs.size(); i++) {
    if (costs[i] > costs[worst_path_index]) {
      worst_path_index = i;
    }
  }
  return worst_path_index;
}

int PathQueue::getBestPathIndex() {
  int best_path_index = 0;
  for (int i = 0; i < costs.size(); i++) {
    if (costs[i] < costs[best_path_index]) {
      best_path_index = i;
    }
  }
  return best_path_index;
}

bool PathQueue::hasPath(Path path) {
  for (Path p : paths) {
    if (p == path) {
      return true;
    }
  }
  return false;
}
