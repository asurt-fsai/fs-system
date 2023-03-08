#pragma once

#include "path.h"

class PathQueue {
public:
  std::vector<Path *> paths; // List of all paths
  std::vector<float> costs;  // List of all costs
  int PATH_QUEUE_LIMIT;      // Maximum number of paths in queue

  PathQueue(int path_queue_limit) { PATH_QUEUE_LIMIT = path_queue_limit; }

  /**
   * @brief Adds a new path to the queue if it is better than the worst path or
   * if the queue is not full
   *
   * @param new_path The new path to add
   * @param cost The cost of the new path
   *
   * @return true If the path was added, false otherwise
   */
  bool addNewPath(Path *new_path, float cost);

  /**
   * @brief Gets the index of the worst path in the queue
   *
   * @return int The index of the worst path
   */
  int getWorstPath();

  /**
   * @brief Gets the index of the best path in the queue
   *
   * @return int The index of the best path
   */
  int getBestPath();
};
