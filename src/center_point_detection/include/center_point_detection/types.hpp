#pragma once

#include "opencv2/core/types.hpp"

struct LineSegment2d
{
  cv::Point2d start;
  cv::Point2d end;
};

/**
 * Enumeration for lane side
 */
enum class LaneSide
{
  LEFT,
  RIGHT,
  UNKNOWN
};

/**
 * Structure representing an intersection point with associated data
 */
struct Intersection
{
  cv::Point2d point = cv::Point2d(0.0, 0.0);
  const LineSegment2d * segment = nullptr;
  cv::Vec2d normal = cv::Vec2d(0.0f, 0.0f);
  LaneSide side = LaneSide::UNKNOWN;
};
