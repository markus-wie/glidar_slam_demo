#pragma once

#include <vector>

#include "center_point_detection/types.hpp"
#include "center_point_detection/ygorclustering/YgorClustering.hpp"
#include "opencv2/core/types.hpp"

namespace dbscan {

void cluster(
  const std::vector<Intersection> & points, std::vector<std::vector<Intersection>> & clusters,
  float epsilon, int minPoints);

}  // namespace dbscan
