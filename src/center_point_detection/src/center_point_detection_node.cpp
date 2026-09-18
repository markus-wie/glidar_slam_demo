#include "center_point_detection/center_point_detection_node.h"

#include "demo_msgs/msg/detail/line__struct.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

CenterPointDetection::CenterPointDetection(const rclcpp::NodeOptions & options)
: rclcpp::Node("center_point_detection", options), m_laneCenterDetector(LaneCenterDetector(*this))
{
  // Parameters
  m_params.m_rayLength = this->declare_parameter("ray_length", 1.2f);
  m_params.m_rayCount = this->declare_parameter("ray_count", 100);
  m_params.m_rayEllipticRatios = this->declare_parameter<std::vector<float>>(
    "ray_elliptic_ratios", std::vector<float>{0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f});
  m_params.m_laneWidth = this->declare_parameter("lane_width", 0.5f);
  m_params.m_maxAngle = this->declare_parameter("max_angle", 360.0f);
  m_params.m_maxDistance = this->declare_parameter("max_dist", 2.0f);
  m_params.m_maxPoints = this->declare_parameter("max_points", 5);
  m_params.m_debugPrint = this->declare_parameter("draw_rays_in_debug_image", false);
  m_params.m_laneMode =
    static_cast<LaneMode>(this->declare_parameter("lane_mode", static_cast<int>(LaneMode::BOTH)));
  m_params.m_debugPrintRays = this->declare_parameter("draw_rays_only_at_iteration_number", 0);
  m_params.m_drawIntersectionPoints = this->declare_parameter("draw_intersection_points", false);
  m_params.m_drawTranslatedPoints = this->declare_parameter("draw_translated_points", false);
  m_params.m_dbscanEps = this->declare_parameter("dbscan_eps", 0.1f);
  m_params.m_dbscanMinPoints = this->declare_parameter("dbscan_min_points", 3);
  m_params.m_dbscanEnable = this->declare_parameter("dbscan_enable", true);
  m_params.m_vehicleWheelbase = this->declare_parameter("vehicle.wheelbase", 0.28);
  m_params.m_vehicleWheelSeparation = this->declare_parameter("vehicle.wheel_separation", 0.26);
  m_params.m_vehicleCenterToCamOffset =
    this->declare_parameter("vehicle.center_to_cam_offset", 0.62);
  m_params.m_vehicleCamToWorldScaleFactor =
    this->declare_parameter("vehicle.cam_to_world_scale_factor", 2.77);

  // Subscribers
  this->m_subLsdLines = this->create_subscription<demo_msgs::msg::LSDandImage>(
    "/line_segment_detection/detected_lines/LSDandImage", rclcpp::SensorDataQoS(),
    std::bind(&CenterPointDetection::lsdLinesCb, this, _1));
  this->m_subTrajectoryPoints = this->create_subscription<demo_msgs::msg::LanePoints>(
    "/trajectory_planner/trajectory_points", 5,
    std::bind(&CenterPointDetection::trajectoryPointsCallback, this, _1));
  this->m_resetCenterPointDetection = this->create_subscription<std_msgs::msg::Bool>(
    "/center_point_detection/reset", 5,
    std::bind(&CenterPointDetection::resetCenterPointDetectionCallback, this, _1));

  // Publishers
  this->m_pubDebugImage = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/center_point_detection/debug_image/compressed", rclcpp::SensorDataQoS());
  this->m_pubCenterPoints =
    this->create_publisher<demo_msgs::msg::LanePoints>("/center_point_detection/center_points", 5);

  // Parameter Callback
  m_callbackHandle = this->add_on_set_parameters_callback(
    std::bind(&CenterPointDetection::parametersCallback, this, std::placeholders::_1));
}

void CenterPointDetection::lsdLinesCb(const demo_msgs::msg::LSDandImage & msg)
{
  try {
    cv_bridge::CvImagePtr inCvImage =
      cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::RGB8);
    cv_bridge::CvImage outCvImage{};

    cv::Mat debugImage = inCvImage->image.clone();

    cv::Size orig_size = debugImage.size();
    m_params.m_roiX = orig_size.width;
    m_params.m_roiY = orig_size.height;

    std::vector<LineSegment2d> lines;
    lines.reserve(msg.lines.detected_lines.size());

    for (const demo_msgs::msg::Line & seg : msg.lines.detected_lines) {
      lines.push_back(LineSegment2d{
        cv::Point2d(
          (seg.x1 - m_params.m_roiX / 2.0) / (m_params.m_roiY / 2.0),
          2.0 - seg.y1 / (m_params.m_roiY / 2.0)),
        cv::Point2d(
          (seg.x2 - m_params.m_roiX / 2.0) / (m_params.m_roiY / 2.0),
          2.0 - seg.y2 / (m_params.m_roiY / 2.0))});
    }

    demo_msgs::msg::LanePoints centerPoints;
    m_laneCenterDetector.process(debugImage, lines, centerPoints);

    outCvImage.image = debugImage;

    // Create publisher message with the modified video stream
    cv::Mat resizedOutImage;
    cv::resize(outCvImage.image, resizedOutImage, cv::Size(), 0.5, 0.5);
    auto msg_new = cv_bridge::CvImage(
                     std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, resizedOutImage)
                     .toCompressedImageMsg();
    // Publish the debug image
    m_pubDebugImage->publish(*msg_new);

    if (!centerPoints.data.empty()) {
      m_pubCenterPoints->publish(centerPoints);
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "std exception: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception");
  }
}

void CenterPointDetection::trajectoryPointsCallback(const demo_msgs::msg::LanePoints & msg)
{
  m_trajectoryPoints = msg;
}

void CenterPointDetection::resetCenterPointDetectionCallback(
  [[maybe_unused]] const std_msgs::msg::Bool & msg)
{
  m_laneCenterDetector.resetLaneOffset();
  m_trajectoryPoints.data.clear();
}

rcl_interfaces::msg::SetParametersResult CenterPointDetection::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto & param : parameters) {
    if (
      param.get_name() == "ray_length" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_rayLength = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "draw_rays_in_debug_image" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_params.m_debugPrint = param.as_bool();
      result.successful = true;
    } else if (
      param.get_name() == "ray_count" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_params.m_rayCount = static_cast<int>(param.as_int());
      result.successful = true;
    } else if (
      param.get_name() == "ray_elliptic_ratios" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      m_params.m_rayEllipticRatios = param.as_double_array();
      result.successful = true;
    } else if (
      param.get_name() == "lane_width" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_laneWidth = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "max_angle" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_maxAngle = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "max_dist" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_maxDistance = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "max_points" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_params.m_maxPoints = static_cast<int>(param.as_int());
      result.successful = true;
    } else if (
      param.get_name() == "lane_mode" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      auto laneMode = static_cast<int>(param.as_int());
      if (laneMode < 0 || laneMode > 2) {
        result.successful = false;
      } else {
        if (laneMode == 0) {
          m_params.m_laneMode = LaneMode::BOTH;
        } else if (laneMode == 1) {
          m_params.m_laneMode = LaneMode::LEFT;
        } else {
          m_params.m_laneMode = LaneMode::RIGHT;
        }
        result.successful = true;
      }
    } else if (
      param.get_name() == "draw_rays_only_at_iteration_number" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_params.m_debugPrintRays = static_cast<int>(param.as_int());
      result.successful = true;
    } else if (
      param.get_name() == "draw_intersection_points" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_params.m_drawIntersectionPoints = param.as_bool();
      result.successful = true;
    } else if (
      param.get_name() == "draw_translated_points" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_params.m_drawTranslatedPoints = param.as_bool();
      result.successful = true;
    } else if (
      param.get_name() == "dbscan_eps" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_dbscanEps = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "dbscan_min_points" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_params.m_dbscanMinPoints = static_cast<int>(param.as_int());
      result.successful = true;
    } else if (
      param.get_name() == "dbscan_enable" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_params.m_dbscanEnable = param.as_bool();
      result.successful = true;
    } else if (
      param.get_name() == "vehicle.wheelbase" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_vehicleWheelbase = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "vehicle.wheel_separation" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_vehicleWheelSeparation = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "vehicle.center_to_cam_offset" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_vehicleCenterToCamOffset = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "vehicle.cam_to_world_scale_factor" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_params.m_vehicleCamToWorldScaleFactor = static_cast<float>(param.as_double());
      result.successful = true;
    } else {
      result.successful = false;
      result.reason = "Invalid parameter or type";
      RCLCPP_WARN(
        this->get_logger(), "center_point_detection: Could not set parameter %s",
        param.get_name().c_str());
      break;
    }
  }
  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(CenterPointDetection)
