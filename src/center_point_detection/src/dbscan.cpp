#include "center_point_detection/dbscan.hpp"

#include <algorithm>
#include <chrono>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace dbscan {

void cluster(
  const std::vector<Intersection> & points, std::vector<std::vector<Intersection>> & clusters,
  float epsilon, int minPoints)
{
  // 2D double points, with 1 int attribute
  using CDat_t = ClusteringDatum<2, double, 0, float, uint16_t, int>;

  constexpr size_t MaxElementsInANode = 6;
  using RTreeParameter_t = boost::geometry::index::rstar<MaxElementsInANode>;
  using RTree_t = boost::geometry::index::rtree<CDat_t, RTreeParameter_t>;

  std::vector<CDat_t> rtreePoints;
  rtreePoints.reserve(points.size());
  std::for_each(
    points.begin(), points.end(), [&rtreePoints, n = 0](const Intersection & intersection) mutable {
      rtreePoints.emplace_back(CDat_t({intersection.point.x, intersection.point.y}, {}, n));
      n++;
    });

  RTree_t rtree(rtreePoints.begin(), rtreePoints.end());

  ::DBSCAN<RTree_t, CDat_t>(rtree, epsilon, minPoints);

  constexpr auto RTreeSpatialQueryGetAll = [](const CDat_t & p) -> bool {
    if (p.CID.IsNoise()) {
      return false;
    }
    if (p.CID.IsUnclassified()) {
      return false;
    }
    return true;
  };

  std::map<ClusterID<typename CDat_t::ClusterIDType_>, std::vector<Intersection>> clusterMap;

  RTree_t::const_query_iterator it;
  it = rtree.qbegin(boost::geometry::index::satisfies(RTreeSpatialQueryGetAll));
  for (; it != rtree.qend(); ++it) {
    clusterMap[it->CID].push_back(points[it->UserData]);
  }

  clusters.clear();
  clusters.reserve(clusterMap.size());
  for (const auto & pair : clusterMap) {
    clusters.push_back(pair.second);
  }
}

}  // namespace dbscan
