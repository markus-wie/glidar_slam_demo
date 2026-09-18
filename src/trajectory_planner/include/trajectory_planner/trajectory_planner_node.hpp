#pragma once

#include <memory>
#include <string>
#include <vector>

#include "demo_msgs/msg/lane_points.hpp"
// #include "demo_msgs/msg/segment.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class TrajectoryPlanner : public rclcpp::Node
{
public:
  explicit TrajectoryPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<demo_msgs::msg::LanePoints>::SharedPtr m_subLanePoints;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subControlTrajectoryPlanning;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel_pub;

  float m_steeringAngleFactor = 1.0f;
  float m_steeringAngleChangeRatio = 0.6f;
  bool m_useSinglePoint = true;
  int m_useSinglePointAtIteration = 1;
  int m_pointAmountForAverage = 3;
  float m_frontAxisOffset = -0.34f;
  float m_backAxisOffset = -0.62f;
  float m_wheelbase = 0.28f;

  float m_lastSteeringAngle = 0.0f;
  bool m_remoteControlActive = false;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_callbackHandle;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void lanePointsCallback(const demo_msgs::msg::LanePoints & msg);

  void remoteControllerCallback(const std_msgs::msg::Bool & msg);

  void calculateAckermannSteering(
    const std::vector<demo_msgs::msg::LanePoint> & lanePoints, float & steeringAngle,
    float & arcLength) const;
};
