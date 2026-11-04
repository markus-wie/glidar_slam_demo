#include "trajectory_planner/trajectory_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace {
struct Point2d
{
  double x;
  double y;
};

struct Line2d
{
  Point2d start;
  Point2d end;
};

Point2d operator+(const Point2d & lhs, const Point2d & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Point2d operator-(const Point2d & lhs, const Point2d & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Point2d operator*(const Point2d & point, double scalar)
{
  return {point.x * scalar, point.y * scalar};
}

double cross(const Point2d & lhs, const Point2d & rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

bool intersectLines(const Line2d & first, const Line2d & second, Point2d & intersection)
{
  const Point2d firstDirection = first.end - first.start;
  const Point2d secondDirection = second.end - second.start;
  const double denominator = cross(firstDirection, secondDirection);
  if (std::abs(denominator) < 1e-9) {
    return false;
  }

  const double firstParameter = cross(second.start - first.start, secondDirection) / denominator;
  intersection = first.start + firstDirection * firstParameter;
  return true;
}
}  // namespace

TrajectoryPlanner::TrajectoryPlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_planner", options)
{
  m_steeringAngleFactor = this->declare_parameter("steering_angle_factor", 1.0);
  m_steeringAngleChangeRatio = this->declare_parameter("steering_angle_change_ratio", 0.6);
  m_useSinglePoint = this->declare_parameter("use_single_point", true);
  m_useSinglePointAtIteration = this->declare_parameter("use_single_point_at_iteration", 1);
  m_pointAmountForAverage = this->declare_parameter("point_amount_for_average", 3);
  m_speed = this->declare_parameter("speed", 1.5);
  m_frontAxisOffset = this->declare_parameter("vehicle.front_axis_offset", -0.34);
  m_backAxisOffset = this->declare_parameter("vehicle.back_axis_offset", -0.62);

  m_wheelbase = std::abs(m_frontAxisOffset - m_backAxisOffset);

  m_subLanePoints = this->create_subscription<demo_msgs::msg::LanePoints>(
    "/center_point_detection/center_points", 10,
    std::bind(&TrajectoryPlanner::lanePointsCallback, this, _1));
  m_subControlTrajectoryPlanning = this->create_subscription<std_msgs::msg::Bool>(
    "/trajectory_planner/control_planning", 10,
    std::bind(&TrajectoryPlanner::remoteControllerCallback, this, _1));

  m_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  m_callbackHandle = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryPlanner::parametersCallback, this, std::placeholders::_1));
}

void TrajectoryPlanner::lanePointsCallback(const demo_msgs::msg::LanePoints & msg)
{
  if (!m_remoteControlActive) {
    return;
  }

  float steeringAngle = m_lastSteeringAngle;
  float arcLength = 0.0f;
  if (!msg.data.empty()) {
    calculateAckermannSteering(msg.data, steeringAngle, arcLength);
    m_lastSteeringAngle = steeringAngle;
  }

  steeringAngle *= m_steeringAngleFactor;

  float steering_angle_rad = steeringAngle * (M_PIf / 180.0f);
  float angular_velocity = (m_speed * std::tan(steering_angle_rad)) / m_wheelbase;

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist.angular.z = angular_velocity;
  twist_msg.twist.linear.x = m_speed;

  m_cmd_vel_pub->publish(twist_msg);
}

void TrajectoryPlanner::calculateAckermannSteering(
  const std::vector<demo_msgs::msg::LanePoint> & lanePoints, float & steeringAngle,
  float & arcLength) const
{
  const int selectedPointIndex =
    std::clamp(m_useSinglePointAtIteration, 0, static_cast<int>(lanePoints.size()) - 1);
  Point2d averagedPoint{0.0, 0.0};

  if (m_useSinglePoint || m_pointAmountForAverage <= 0) {
    averagedPoint = Point2d{
      lanePoints[static_cast<std::size_t>(selectedPointIndex)].x,
      lanePoints[static_cast<std::size_t>(selectedPointIndex)].y};
  } else {
    const int pointCount =
      std::min(static_cast<int>(lanePoints.size()) - 1, std::max(m_pointAmountForAverage, 0));
    if (pointCount <= 0) {
      averagedPoint = Point2d{lanePoints.front().x, lanePoints.front().y};
    } else {
      for (int i = 1; i <= pointCount; ++i) {
        averagedPoint = averagedPoint + Point2d{
                                          lanePoints[static_cast<std::size_t>(i)].x,
                                          lanePoints[static_cast<std::size_t>(i)].y};
      }
      averagedPoint = averagedPoint * (1.0 / static_cast<double>(pointCount));
    }
  }

  const Point2d frontAxis{0.0, m_frontAxisOffset};
  const Point2d target{averagedPoint.x, averagedPoint.y};
  const Point2d midpoint = (target + frontAxis) * 0.5;
  const Point2d targetDirection = target - frontAxis;
  const Point2d perpendicularDirection{-targetDirection.y, targetDirection.x};
  const Line2d intersectionLine{
    midpoint - perpendicularDirection, midpoint + perpendicularDirection};
  const Line2d backAxisLine{{-1.0, m_backAxisOffset}, {1.0, m_backAxisOffset}};

  Point2d circleCenter{0.0, 0.0};
  if (!intersectLines(intersectionLine, backAxisLine, circleCenter)) {
    steeringAngle = m_lastSteeringAngle;
    arcLength = 0.0f;
    return;
  }

  const float radius = std::abs(static_cast<float>(circleCenter.x));
  const float unsignedSteeringAngle =
    180.0f / static_cast<float>(M_PI) *
    std::asin(m_wheelbase / std::sqrt(m_wheelbase * m_wheelbase + radius * radius));
  const float steeringAngleSign = circleCenter.x > 0.0 ? -1.0f : 1.0f;
  const float signedSteeringAngle = steeringAngleSign * unsignedSteeringAngle;

  steeringAngle = m_lastSteeringAngle * (1.0f - m_steeringAngleChangeRatio) +
                  signedSteeringAngle * m_steeringAngleChangeRatio;
  const float deltaTheta = std::abs(steeringAngle) * static_cast<float>(M_PI) / 180.0f;
  arcLength = radius * deltaTheta;
}

void TrajectoryPlanner::remoteControllerCallback(const std_msgs::msg::Bool & msg)
{
  m_remoteControlActive = msg.data;
}

rcl_interfaces::msg::SetParametersResult TrajectoryPlanner::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (
      parameter.get_name() == "steering_angle_factor" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_steeringAngleFactor = static_cast<float>(parameter.as_double());
    } else if (
      parameter.get_name() == "steering_angle_change_ratio" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_steeringAngleChangeRatio = static_cast<float>(parameter.as_double());
    } else if (
      parameter.get_name() == "use_single_point" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_useSinglePoint = parameter.as_bool();
    } else if (
      parameter.get_name() == "use_single_point_at_iteration" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_useSinglePointAtIteration = static_cast<int>(parameter.as_int());
    } else if (
      parameter.get_name() == "point_amount_for_average" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_pointAmountForAverage = static_cast<int>(parameter.as_int());
    } else if (
      parameter.get_name() == "speed" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_speed = static_cast<float>(parameter.as_double());
    } else if (
      parameter.get_name() == "vehicle.front_axis_offset" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_frontAxisOffset = static_cast<float>(parameter.as_double());
      m_wheelbase = std::abs(m_frontAxisOffset - m_backAxisOffset);
    } else if (
      parameter.get_name() == "vehicle.back_axis_offset" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_backAxisOffset = static_cast<float>(parameter.as_double());
      m_wheelbase = std::abs(m_frontAxisOffset - m_backAxisOffset);
    } else {
      result.successful = false;
      result.reason = "Unknown or invalid parameter: " + parameter.get_name();
      break;
    }
  }

  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryPlanner)
