#include "center_point_detection/lane_center_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include "center_point_detection/center_point_detection_node.h"
#include "center_point_detection/dbscan.hpp"
#include "center_point_detection/parameters.h"
#include "opencv2/core/matx.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

LaneCenterDetector::LaneCenterDetector(rclcpp::Node & lf) : m_node(&lf)
{
}

namespace {
int camToPixelX(double x, int roiX, int roiY)
{
  return static_cast<int>(0.5 * x * roiY + 0.5 * roiX);
}

int camToPixelY(double y, int roiY)
{
  return static_cast<int>(-(0.5 * roiY) * (y - 2.0));
}

double signedAngle(const cv::Vec2d & first, const cv::Vec2d & second)
{
  const double firstNorm = cv::norm(first);
  const double secondNorm = cv::norm(second);
  if (firstNorm == 0.0 || secondNorm == 0.0) {
    return 0.0;
  }
  const double cosine = std::clamp(first.dot(second) / (firstNorm * secondNorm), -1.0, 1.0);
  return std::acos(cosine);
}

double directionAngle(const cv::Vec2d & vector)
{
  return std::atan2(vector[1], vector[0]);
}

double absoluteAngle(const cv::Vec2d & first, const cv::Vec2d & second)
{
  const double firstNorm = cv::norm(first);
  const double secondNorm = cv::norm(second);
  if (firstNorm == 0.0 || secondNorm == 0.0) {
    return 0.0;
  }
  const double cosine =
    std::clamp(std::abs(first.dot(second) / (firstNorm * secondNorm)), 0.0, 1.0);
  return std::acos(cosine);
}

bool intersects(const LineSegment2d & first, const LineSegment2d & second, cv::Point2d & point)
{
  const cv::Point2d firstDirection = first.end - first.start;
  const cv::Point2d secondDirection = second.end - second.start;
  const double denominator =
    firstDirection.x * secondDirection.y - firstDirection.y * secondDirection.x;
  if (std::abs(denominator) < 1e-9) {
    return false;
  }
  const cv::Point2d offset = second.start - first.start;
  const double firstParameter =
    (offset.x * secondDirection.y - offset.y * secondDirection.x) / denominator;
  const double secondParameter =
    (offset.x * firstDirection.y - offset.y * firstDirection.x) / denominator;
  if (
    firstParameter < 0.0 || firstParameter > 1.0 || secondParameter < 0.0 ||
    secondParameter > 1.0) {
    return false;
  }
  point = first.start + firstDirection * firstParameter;
  return true;
}

double ccw(const cv::Point2d & point, const cv::Point2d & first, const cv::Point2d & second)
{
  return (first.x - point.x) * (second.y - point.y) - (first.y - point.y) * (second.x - point.x);
}
}  // namespace

void LaneCenterDetector::process(
  cv::Mat & debug, const std::vector<LineSegment2d> & lines,
  demo_msgs::msg::LanePoints & lanePoints)
{
  // Define the point from where the sampling rays will be casted.
  cv::Vec2d currentOrigin(m_laneOffset, 0.0f);

  // Starting point of the next iteration
  cv::Vec2d nextOrigin(0.0f, 0.0f);

  // Assume that the street is directly in front of the car
  cv::Vec2d currentSearchDirection(0.0f, 1.0f);
  // Starting search direction of the next iteration
  cv::Vec2d nextSearchDirection;

  // The sequence of points who approximate the line to be followed
  int searchSteps = m_params.m_maxPoints + 1;  // +1 for the initial origin

  // Continue until no lane is found or the found lane is long enough
  bool laneFound = true;

  m_originFound = false;

  m_iterationNumber = 0;

  while (laneFound && m_iterationNumber < searchSteps &&
         cv::norm(currentOrigin) < m_params.m_maxDistance) {
    laneFound = searchLaneCenter(
      lines, currentOrigin, currentSearchDirection, debug, nextOrigin, nextSearchDirection);

    if (laneFound) {
      demo_msgs::msg::LanePoint & point = lanePoints.data.emplace_back();
      point.x = nextOrigin[0];
      point.y = nextOrigin[1];
      currentOrigin = nextOrigin;
      currentSearchDirection = nextSearchDirection;
      m_iterationNumber++;
    } else {
      RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 5000, "No Lane was found");
      break;
    }
  }

  // Vehicle dimensions in the camera space used for visualization
  const double carLength = m_params.m_vehicleWheelbase / m_params.m_vehicleCamToWorldScaleFactor;
  const double carWidth =
    m_params.m_vehicleWheelSeparation / m_params.m_vehicleCamToWorldScaleFactor;

  // Extra image height to be added for visualization based on the camera offset to the car
  const double extraImageHeight =
    m_params.m_vehicleCenterToCamOffset / m_params.m_vehicleCamToWorldScaleFactor + carLength / 2.0;

  // Calculate the extra height needed for the debug image based on the camera offset to the car
  const float conversionFactor = 0.5f * static_cast<float>(m_params.m_roiY);
  const double extraHeightPix = static_cast<int>(conversionFactor * extraImageHeight);

  // Add black area at the bottom of the image to account for the camera offset to the car
  cv::Mat blackArea = cv::Mat::zeros(extraHeightPix, debug.cols, debug.type());
  cv::vconcat(debug, blackArea, debug);

  // Draw text
  std::string str =
    "Lane Points found: " + std::to_string(
                              std::max(static_cast<int>(lanePoints.data.size()), 1) -
                              1);  // -1 because the first point is the origin
  cv::putText(
    debug, str, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2, false);

  // Draw Center Points
  DrawConnectedPoints(debug, lanePoints, cv::Scalar(255, 1, 255, 255), cv::Scalar(0, 255, 0));

  const float cam_to_world_scale_factor = m_params.m_vehicleCamToWorldScaleFactor;
  const float center_to_cam_offset = m_params.m_vehicleCenterToCamOffset;

  // Draw Car
  cv::rectangle(
    debug,
    cv::Point(
      camToPixelX(carWidth / 2.0, m_params.m_roiX, m_params.m_roiY),
      camToPixelY(
        -center_to_cam_offset / cam_to_world_scale_factor + carLength / 2.0, m_params.m_roiY)),
    cv::Point(
      camToPixelX(-carWidth / 2.0, m_params.m_roiX, m_params.m_roiY),
      camToPixelY(
        -center_to_cam_offset / cam_to_world_scale_factor - carLength / 2.0, m_params.m_roiY)),
    cv::Scalar(50, 50, 50), cv::FILLED);
  auto * casted_node = dynamic_cast<CenterPointDetection *>(m_node);
  // Draw Trajectory Points
  DrawConnectedPoints(
    debug, casted_node->m_trajectoryPoints, cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0));
}
void LaneCenterDetector::DrawConnectedPoints(
  cv::Mat & image, const demo_msgs::msg::LanePoints & points, const cv::Scalar & pointColor,
  const cv::Scalar & lineColor)
{
  if (!points.data.empty()) {
    int xBefore = camToPixelX(points.data.at(0).x, m_params.m_roiX, m_params.m_roiY);
    int yBefore = camToPixelY(points.data.at(0).y, m_params.m_roiY);
    for (auto it = points.data.begin() + 1; it != points.data.end(); it++) {
      int currentX = camToPixelX(it->x, m_params.m_roiX, m_params.m_roiY);
      int currentY = std::max(camToPixelY(it->y, m_params.m_roiY), 2);
      cv::line(image, cv::Point(xBefore, yBefore), cv::Point(currentX, currentY), lineColor, 1);
      cv::circle(image, cv::Point(currentX, currentY), 2, pointColor, cv::FILLED);
      xBefore = currentX;
      yBefore = currentY;
    }
  }
}

bool LaneCenterDetector::searchLaneCenter(
  const std::vector<LineSegment2d> & laneSegments, const cv::Vec2d & currentOrigin,
  const cv::Vec2d & currentSearchDirection, cv::Mat & debugImg, cv::Vec2d & nextOrigin,
  cv::Vec2d & nextSearchDirection)
{
  std::vector<cv::Vec4d> samples;
  createSamplingRays(currentOrigin, currentSearchDirection, debugImg, samples);

  std::vector<Intersection> intersections;
  if (!getClosestIntersections(samples, laneSegments, currentSearchDirection, intersections)) {
    return false;
  }

  filterLeftRightLanes(intersections, currentOrigin, currentSearchDirection);

  std::vector<Intersection> leftIntersections;
  std::vector<Intersection> rightIntersections;
  clusterIntersections(intersections, leftIntersections, rightIntersections);

  if (m_params.m_laneMode == LaneMode::LEFT) {
    rightIntersections.clear();
  } else if (m_params.m_laneMode == LaneMode::RIGHT) {
    leftIntersections.clear();
  }

  if (!m_params.m_drawTranslatedPoints) {
    drawIntersectionsDebug(leftIntersections, debugImg, cv::Scalar(255, 0, 0));
    drawIntersectionsDebug(rightIntersections, debugImg, cv::Scalar(0, 255, 0));
  }

  createTranslatedPoints(leftIntersections);
  createTranslatedPoints(rightIntersections);

  if (m_params.m_drawTranslatedPoints) {
    drawIntersectionsDebug(leftIntersections, debugImg, cv::Scalar(255, 0, 0));
    drawIntersectionsDebug(rightIntersections, debugImg, cv::Scalar(0, 255, 0));
  }

  std::vector<Intersection> translatedIntersections;
  translatedIntersections.reserve(leftIntersections.size() + rightIntersections.size());
  translatedIntersections.insert(
    translatedIntersections.end(), leftIntersections.begin(), leftIntersections.end());
  translatedIntersections.insert(
    translatedIntersections.end(), rightIntersections.begin(), rightIntersections.end());

  if (translatedIntersections.size() <= 0) {
    return false;
  }

  cv::Vec2d averagedPoint;
  for (const Intersection & intersection : translatedIntersections) {
    averagedPoint += cv::Vec2d(intersection.point);
  }
  averagedPoint = averagedPoint / static_cast<float>(translatedIntersections.size());

  if (!m_originFound) {
    nextOrigin = cv::Vec2d(averagedPoint[0], 0.0f);
    nextSearchDirection = currentSearchDirection;
    m_laneOffset = averagedPoint[0];
    m_originFound = true;
    return true;
  }

  auto originBefore = cv::Vec2d(nextOrigin[0], nextOrigin[1]);
  nextOrigin = averagedPoint;

  // This step ensures that every point has a positive y value.
  // This is an unfortunate necessity because the rays would point in a completely wrong direction
  // otherwise, which can lead to very bad findings in later iterations
  nextOrigin[1] = std::max(nextOrigin[1], 0.0);

  nextSearchDirection =
    cv::normalize(cv::Vec2d(nextOrigin[0] - originBefore[0], nextOrigin[1] - originBefore[1]));

  float angleChange = signedAngle(currentSearchDirection, nextSearchDirection);

  // Limit the maximum angle change to a bit more than 90 degrees. When this happens, the detection
  // definately went wrong. +0.1 since the very first iteration can lead to a 90 degree turn in
  // curves.
  return std::abs(angleChange) <= (M_PI_2f + 0.1f);
}

void LaneCenterDetector::createSamplingRays(
  const cv::Vec2d & currentOrigin, const cv::Vec2d & currentSearchDirection,
  const cv::Mat & debugImg, std::vector<cv::Vec4d> & samples) const
{
  const int rayCount = m_params.m_rayCount;

  samples.reserve(rayCount);

  const cv::Vec2d currentSearchDirectionNormal(
    -currentSearchDirection[1], currentSearchDirection[0]);

  const float ellipticRatio =
    m_params.m_rayEllipticRatios.at(m_iterationNumber % m_params.m_rayEllipticRatios.size());

  // Create sample rays in a half ellipse. We want to use an ellipse to avoid taking too many
  // points from the far into consideration for close points.
  // Define the two radii of the ellipse:
  const float verticalEllipseRadius = m_params.m_rayLength * ellipticRatio;
  const float horizontalEllipseRadius = m_params.m_rayLength;

  std::array<float, 2> samplingRange = {static_cast<float>(0.0), static_cast<float>(M_PI)};
  for (int i = 0; i < rayCount; i++) {
    float angle = samplingRange[1] * static_cast<float>(i) / (static_cast<float>(rayCount) - 1) +
                  samplingRange[0];

    double x = currentOrigin[0] +
               verticalEllipseRadius * std::sin(angle) * currentSearchDirection[0] +
               horizontalEllipseRadius * std::cos(angle) * currentSearchDirectionNormal[0];
    double y = currentOrigin[1] +
               verticalEllipseRadius * std::sin(angle) * currentSearchDirection[1] +
               horizontalEllipseRadius * std::cos(angle) * currentSearchDirectionNormal[1];

    samples.emplace_back(currentOrigin[0], currentOrigin[1], x, y);

    if (m_params.m_debugPrint && m_params.m_debugPrintRays == m_iterationNumber) {
      cv::line(
        debugImg,
        cv::Point(
          camToPixelX(currentOrigin[0], m_params.m_roiX, m_params.m_roiY),
          camToPixelY(currentOrigin[1], m_params.m_roiY)),
        cv::Point(
          camToPixelX(x, m_params.m_roiX, m_params.m_roiY), camToPixelY(y, m_params.m_roiY)),
        cv::Scalar(0, 255, 255, 64), 2);
    }
  }
}

bool LaneCenterDetector::getClosestIntersections(
  const std::vector<cv::Vec4d> & samples, const std::vector<LineSegment2d> & laneSegments,
  const cv::Vec2d & currentSearchDirection, std::vector<Intersection> & intersections) const
{
  intersections.reserve(samples.size());

  float maxAngularDifference = m_params.m_maxAngle / 360.0f * 2.0f * CV_PI;

  for (const auto & sample : samples) {
    const LineSegment2d sampleLine{
      cv::Point2d(sample[0], sample[1]), cv::Point2d(sample[2], sample[3])};

    std::vector<const LineSegment2d *> intersectedLines;
    for (const LineSegment2d & laneSegment : laneSegments) {
      cv::Point2d intersection;
      if (intersects(sampleLine, laneSegment, intersection)) {
        intersectedLines.push_back(&laneSegment);
      }
    }

    Intersection intersection;
    if (getClosestIntersection(intersectedLines, sampleLine, intersection)) {
      const cv::Vec2d startPoint = cv::Vec2d(intersection.segment->start);
      const cv::Vec2d endPoint = cv::Vec2d(intersection.segment->end);

      cv::Vec2d lineDir = cv::normalize(endPoint - startPoint);

      if (absoluteAngle(lineDir, currentSearchDirection) > maxAngularDifference) {
        continue;
      }

      intersections.emplace_back(intersection);
    }
  }
  return !intersections.empty();
}

bool LaneCenterDetector::getClosestIntersection(
  const std::vector<const LineSegment2d *> & intersectedLines, const LineSegment2d & sampleLine,
  Intersection & closestIntersection)
{
  const cv::Point2d & currentOriginPoint = sampleLine.start;
  double minDistance = std::numeric_limits<double>::max();
  bool foundClosest = false;
  for (const LineSegment2d * intersectedLine : intersectedLines) {
    cv::Point2d intersection;
    if (!intersects(sampleLine, *intersectedLine, intersection)) {
      continue;
    }

    // The distance between the origin and the intersection
    const double distance = cv::norm((currentOriginPoint - intersection));

    if (distance < minDistance) {
      minDistance = distance;
      closestIntersection = {intersection, intersectedLine};
      foundClosest = true;
    }
  }
  return foundClosest;
}

void LaneCenterDetector::filterLeftRightLanes(
  std::vector<Intersection> & intersections, const cv::Vec2d & currentOrigin,
  const cv::Vec2d & currentSearchDirection)
{
  for (Intersection & intersection : intersections) {
    const cv::Vec2d startPoint = cv::Vec2d(intersection.segment->start);
    const cv::Vec2d endPoint = cv::Vec2d(intersection.segment->end);
    cv::Vec2d lineDir = cv::normalize(endPoint - startPoint);
    auto normal2d = cv::Vec2d(-lineDir[1], lineDir[0]);
    // If the origin is on the left side of the line segment,
    // mirror the normal, so that it points towards origin, aka towards the lane center
    if (ccw(currentOrigin, intersection.segment->start, intersection.segment->end) < 0) {
      normal2d[0] = -normal2d[0];
      normal2d[1] = -normal2d[1];
    }

    intersection.normal = normal2d;

    double normalAngleToSearchDirection =
      directionAngle(currentSearchDirection) - directionAngle(normal2d);

    if (normalAngleToSearchDirection > 0) {
      intersection.side = LaneSide::RIGHT;
    } else {
      intersection.side = LaneSide::LEFT;
    }
  }
}

void LaneCenterDetector::clusterIntersections(
  const std::vector<Intersection> & intersections, std::vector<Intersection> & leftIntersections,
  std::vector<Intersection> & rightIntersections) const
{
  if (m_params.m_dbscanEnable) {
    std::vector<std::vector<Intersection>> clusters;
    dbscan::cluster(intersections, clusters, m_params.m_dbscanEps, m_params.m_dbscanMinPoints);

    std::sort(
      clusters.begin(), clusters.end(),
      [](const std::vector<Intersection> & a, const std::vector<Intersection> & b) {
        return a.size() > b.size();
      });

    for (int i = 0; i < std::min<int>(clusters.size(), 2); i++) {
      int leftCount = 0;
      int rightCount = 0;
      for (const Intersection & intersection : clusters[i]) {
        if (intersection.side == LaneSide::LEFT) {
          leftCount++;
        } else if (intersection.side == LaneSide::RIGHT) {
          rightCount++;
        }
      }

      if (leftCount >= rightCount) {
        leftIntersections.insert(leftIntersections.end(), clusters[i].begin(), clusters[i].end());
      } else {
        rightIntersections.insert(rightIntersections.end(), clusters[i].begin(), clusters[i].end());
      }
    }

    std::for_each(
      leftIntersections.begin(), leftIntersections.end(), [](Intersection & intersection) {
        intersection.side = LaneSide::LEFT;
      });

    std::for_each(
      rightIntersections.begin(), rightIntersections.end(), [](Intersection & intersection) {
        intersection.side = LaneSide::RIGHT;
      });
  } else {
    for (const Intersection & intersection : intersections) {
      if (intersection.side == LaneSide::LEFT) {
        leftIntersections.emplace_back(intersection);
      } else if (intersection.side == LaneSide::RIGHT) {
        rightIntersections.emplace_back(intersection);
      }
    }
  }
}

void LaneCenterDetector::createTranslatedPoints(std::vector<Intersection> & intersections) const
{
  for (auto & intersection : intersections) {
    // translate intersection along the normal vector by half the defined lane width
    intersection.point =
      cv::Vec2d(intersection.point) + intersection.normal * m_params.m_laneWidth / 2.0f;
  }
}

void LaneCenterDetector::drawIntersectionsDebug(
  const std::vector<Intersection> & intersections, const cv::Mat & debugImg,
  const cv::Scalar color) const
{
  // draw points for visualization
  if (m_params.m_drawIntersectionPoints) {
    for (const auto & intersection : intersections) {
      auto toPrint = cv::Vec2d(intersection.point);
      int px = camToPixelX(toPrint[0], m_params.m_roiX, m_params.m_roiY);
      int py = camToPixelY(toPrint[1], m_params.m_roiY);
      cv::circle(debugImg, cv::Point(px, py), 1, color, cv::FILLED);
    }
  }
}

void LaneCenterDetector::resetLaneOffset()
{
  m_laneOffset = 0.0f;
}
