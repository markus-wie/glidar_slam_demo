#pragma once

#include <memory>
#include <vector>

#include "center_point_detection/parameters.h"
#include "center_point_detection/types.hpp"
#include "demo_msgs/msg/detail/lane_points__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * This class detects the center points of the lane.
 * The algorithm takes lines detected from the camera image as input and calculates a list of center
 * points.
 */
class LaneCenterDetector
{
public:
  /**
   * Initialize the lane detector with standard parameters.
   */
  explicit LaneCenterDetector(rclcpp::Node & lf);

  LaneCenterDetector() = delete;

  /**
   * Destructor
   */
  ~LaneCenterDetector() = default;

  /**
   * Process a single frame for lane center points.
   *
   * Public call to the algorithm. Should be called once per frame. See the description at the top
   * of the header file for a reference how this works
   *
   * @param debug A debug image for visualization.
   * @param lines The detected lane lines.
   * @param lanePoints The output lane center points.
   */
  void process(
    cv::Mat & debug, const std::vector<LineSegment2d> & lines,
    demo_msgs::msg::LanePoints & lanePoints);

  /**
   * Reset lane offset.
   */
  void resetLaneOffset();

private:
  Parameters & m_params = Parameters::getInstance();

  /**
   * Last lane offset to be set as origin from where the sampling rays will be casted
   */
  float m_laneOffset = 0.0f;

  /**
   * Keeps track of the current iteration number for debug purposes.
   */
  int m_iterationNumber = 0;

  /**
   * Whether the first point per frame ahas been found. That first point is used as the initial
   * origin.
   */
  bool m_originFound = false;

  /**
   * ROS Node Reference for logging
   */
  rclcpp::Node * m_node;

  /**
   * This function takes the street data, a 2D point and a 2D direction to
   * calculate the next lane center point and the direction of the lane.
   * By writing the found point and its direction to the respective
   * parameters, the lane can be computed iteratively.
   *
   * @param laneSegments The detected lane segments.
   * @param currentOrigin The position from where the next lane point will
   * be searched.
   * @param currentSearchDirection The direction in which the next lane point
   * is to be looked for.
   * @param debugImg A debug image for visualization.
   * @param nextOrigin The found lane point will be written to this reference.
   * @param nextSearchDirection The found lane direction will be written
   * to this reference.
   * @return Return true if a lane point was found.
   */
  bool searchLaneCenter(
    const std::vector<LineSegment2d> & laneSegments, const cv::Vec2d & currentOrigin,
    const cv::Vec2d & currentSearchDirection, cv::Mat & debugImg, cv::Vec2d & nextOrigin,
    cv::Vec2d & nextSearchDirection);

  /** Draw a line of connected points on a cv::Mat
   *
   * @param image The image where the points will be drawn
   * @param points The points to be drawn
   * @param pointColor The color of the points
   * @param lineColor The color of the lines connecting the points
   */
  void DrawConnectedPoints(
    cv::Mat & image, const demo_msgs::msg::LanePoints & points, const cv::Scalar & pointColor,
    const cv::Scalar & lineColor);

  /**
   * Create sampling rays in an elliptical pattern.
   *
   * The rays are created in an elliptical pattern spanning 180 degrees towards the
   * currentSearchDirection starting from the currentOrigin. The amount and length of the rays can
   * be configured.
   *
   * @param currentOrigin The origin from where the rays will be casted.
   * @param currentSearchDirection The direction in which the search is oriented.
   * @param debugImg A debug image for visualization.
   * @param samples The resulting rays as line segments.
   */
  void createSamplingRays(
    const cv::Vec2d & currentOrigin, const cv::Vec2d & currentSearchDirection,
    const cv::Mat & debugImg, std::vector<cv::Vec4d> & samples) const;

  /**
   * Get the closest intersections for the given samples.
   *
   * @param samples The samples to check for intersections.
   * @param laneSegments The lane segments to use for intersection checks.
   * @param currentSearchDirection The direction in which the search is oriented.
   * @param intersections A list of both the intersection point and the segment.
   * @return True if at least one intersection was found.
   */
  bool getClosestIntersections(
    const std::vector<cv::Vec4d> & samples, const std::vector<LineSegment2d> & laneSegments,
    const cv::Vec2d & currentSearchDirection, std::vector<Intersection> & intersections) const;

  /**
   * Get the closest intersection from a list of intersected lane segments.
   *
   * @param intersectedLines The list of intersected lane segments.
   * @param sampleLine The sample line that was used to find the intersections.
   * @param closestIntersection The resulting closest intersection including both the point and the
   * segment.
   * @return True if a closest intersection was found.
   */
  static bool getClosestIntersection(
    const std::vector<const LineSegment2d *> & intersectedLines, const LineSegment2d & sampleLine,
    Intersection & closestIntersection);

  /**
   * Assign to each intersection whether they belong to the left or right lane markings.
   *
   * The function determines for each intersection whether it belongs to the left or right lane
   * marking. The result is written back to the side attribute of each intersection.
   *
   * @param intersections The list of intersections to filter. The side attribute will be modified
   * in place.
   * @param currentOrigin The origin from where the search was started.
   * @param currentSearchDirection The direction in which the search was oriented.
   */
  static void filterLeftRightLanes(
    std::vector<Intersection> & intersections, const cv::Vec2d & currentOrigin,
    const cv::Vec2d & currentSearchDirection);

  /**
   * Improve the intersections by clustering them and removing outliers.
   *
   * The function clusters all intersections and filters for the at most two biggest clusters and
   * counts the occurrences of left and right intersections in each cluster. The cluster is then
   * assigned to the lane side with the most occurrences. Finally, the points are translated towards
   * the lane center.
   * The clustering removes most outliers and improves the stability of the algorithm at the cost of
   * significant computational overhead. The clustering can be enabled/disabled via the
   * dbscan_enable parameter. If disabled, no clustering is performed and the lane side is assigned
   * directly.
   *
   * @param intersections The list of intersections to improve. The side attribute will be modified
   * in place.
   * @param leftIntersections The resulting left intersections.
   * @param rightIntersections The resulting right intersections.
   */
  void clusterIntersections(
    const std::vector<Intersection> & intersections, std::vector<Intersection> & leftIntersections,
    std::vector<Intersection> & rightIntersections) const;

  /**
   * Draw intersections on a debug image.
   *
   * @param intersections The intersections to draw.
   * @param debugImg The debug image where the intersections will be drawn on.
   * @param color The color of the drawn intersections.
   */
  void drawIntersectionsDebug(
    const std::vector<Intersection> & intersections, const cv::Mat & debugImg,
    cv::Scalar color = cv::Scalar(255, 255, 255)) const;

  /**
   * This function takes the found intersections and translates them to the lane center.
   *
   * Translates each intersection point along the normal of the intersected segment towards the
   * lane center by some configurable amount. The resulting points will mostly be in the middle of
   * the road.
   *
   * @param intersections A list of intersections. The points will be modified in place.
   */
  void createTranslatedPoints(std::vector<Intersection> & intersections) const;
};
