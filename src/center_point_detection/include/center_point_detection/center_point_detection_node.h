#pragma once

#include <memory>
#include <string>
#include <vector>

#include "center_point_detection/lane_center_detector.h"
#include "center_point_detection/parameters.h"
#include "cv_bridge/cv_bridge.hpp"
#include "demo_msgs/msg/lane_point.hpp"
#include "demo_msgs/msg/lane_points.hpp"
#include "demo_msgs/msg/line_segment.hpp"
#include "demo_msgs/msg/ls_dand_image.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

class CenterPointDetection : public rclcpp::Node
{
public:
  explicit CenterPointDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~CenterPointDetection() = default;

  /**
   * Callback function to render trajectory points.
   */
  void trajectoryPointsCallback(const demo_msgs::msg::LanePoints & msg);

  /**
   * Callback function to reset the center point detection.
   */
  void resetCenterPointDetectionCallback(const std_msgs::msg::Bool & msg);

  /**
   * Callback function for receiving the detected lines of the line segment detection node.
   *
   * @param msg The custom lineSegment message of the topic
   * "/line_segment_detection/detected_lines/LSDandImage"
   */
  void lsdLinesCb(const demo_msgs::msg::LSDandImage & msg);

  // these variables are needed for rendering
  demo_msgs::msg::LanePoints m_trajectoryPoints;

private:
  /**
   * This module tries to detect a lane in the projected line segments.
   */
  LaneCenterDetector m_laneCenterDetector;

  Parameters & m_params = Parameters::getInstance();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr m_callbackHandle;

  // Subscribers
  rclcpp::Subscription<demo_msgs::msg::LSDandImage>::SharedPtr m_subLsdLines;
  rclcpp::Subscription<demo_msgs::msg::LanePoints>::SharedPtr m_subTrajectoryPoints;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_resetCenterPointDetection;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pubDebugImage;
  rclcpp::Publisher<demo_msgs::msg::LanePoints>::SharedPtr m_pubCenterPoints;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::TimerBase::SharedPtr crash_timer_;
};
