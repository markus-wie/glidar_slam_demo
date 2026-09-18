#include "image_processing/image_processing_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>

#include "cv_bridge/cv_bridge.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"

namespace image_processing {

namespace {

constexpr int BEV_MASK_KERNEL_SIZE = 7;

rcl_interfaces::msg::ParameterDescriptor integerRangeDescriptor(
  int64_t from, int64_t to, int64_t step)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = step;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor floatingPointRangeDescriptor(
  double from, double to, double step)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = step;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

}  // namespace

ImageProcessingNode::ImageProcessingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_processing", options)
{
  // Parameters
  m_printTimings = this->declare_parameter("print_timings", false);
  m_roiTop = static_cast<float>(this->declare_parameter("ROI_Top", 0.0));
  m_roiBottom = static_cast<float>(this->declare_parameter("ROI_Bottom", 0.0));
  m_roiRight = static_cast<float>(this->declare_parameter("ROI_Right", 0.0));
  m_roiLeft = static_cast<float>(this->declare_parameter("ROI_Left", 0.0));
  m_chainApproxEpsilon = static_cast<float>(this->declare_parameter("chain_approx_epsilon", 1.8));
  m_applyContoursOptimization = this->declare_parameter("apply_contours_optimization", true);
  m_cameraRollDeg = static_cast<float>(this->declare_parameter("roll", 0.0));
  m_cameraPitchDeg = static_cast<float>(this->declare_parameter("pitch", 90.0));
  m_cameraYawDeg = static_cast<float>(this->declare_parameter("yaw", 0.0));
  m_bevScaleFactor = static_cast<float>(this->declare_parameter("bev_scale_factor", 0.1));
  m_adaptiveThresholdblockSize =
    this->declare_parameter("adaptive_threshold_block_size", 51, integerRangeDescriptor(3, 201, 2));
  m_adaptiveThresholdC = this->declare_parameter(
    "adaptive_threshold_c", -20.0, floatingPointRangeDescriptor(-200.0, 200.0, 1.0));

  // Publishers
  lineSegmentPublisher = this->create_publisher<demo_msgs::msg::LSDandImage>(
    "/line_segment_detection/detected_lines/LSDandImage", rclcpp::SensorDataQoS());
  debugWarpedImagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/line_segment_detection/warped_image/debug_image/compressed", rclcpp::SensorDataQoS());
  debugDetectedLinesPublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/line_segment_detection/detected_lines/debug_image/compressed", rclcpp::SensorDataQoS());

  // Subscribers
  m_cameraImageSubscriber.subscribe(this, "/camera/color/image_raw", 1);
  m_cameraInfoSubscriber.subscribe(this, "/camera/color/camera_info", 1);
  m_cameraSynchronizer =
    std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(CameraSyncPolicy(10));
  m_cameraSynchronizer->connectInput(m_cameraImageSubscriber, m_cameraInfoSubscriber);
  m_cameraSynchronizer->registerCallback(std::bind(
    &ImageProcessingNode::cameraCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Parameters Callback
  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ImageProcessingNode::parametersCallback, this, std::placeholders::_1));
}

void ImageProcessingNode::cameraCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

  if (cameraInfoContentChanged(*info)) {
    RCLCPP_INFO(this->get_logger(), "Camera info changed, updating camera matrix.");
    updateCameraMatrix(*info);
    m_updateWarping = true;
    m_lastCameraInfo = *info;
  }

  if (m_updateWarping) {
    RCLCPP_INFO(this->get_logger(), "Updating BEV matrix.");
    updateWarping(info);
    m_updateWarping = false;
  }

  processImage(msg);

  if (m_printTimings) {
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Total processing time: %.2f ms", elapsed.count());
  }
}

bool ImageProcessingNode::cameraInfoContentChanged(const sensor_msgs::msg::CameraInfo & info) const
{
  return info.width != m_lastCameraInfo.width || info.height != m_lastCameraInfo.height ||
         info.k != m_lastCameraInfo.k;
}

void ImageProcessingNode::processImage(sensor_msgs::msg::Image::ConstSharedPtr cameraImageMsg)
{
  cv::Mat currentImage;

  try {
    cv_bridge::CvImagePtr current_frame =
      cv_bridge::toCvCopy(cameraImageMsg, sensor_msgs::image_encodings::RGB8);

    currentImage = current_frame->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Something went wrong...: %s", e.what());
  }

  warpToBirdsEyeView(currentImage);

  std::future<void> debugPublishBirdseyeFuture =
    publishCompressedImage(currentImage, cameraImageMsg->header, debugWarpedImagePublisher);

  detectLineSegments(currentImage);

  debugPublishBirdseyeFuture.wait();

  demo_msgs::msg::LineSegment segMsg;
  cv::Mat segmentedImage;
  findLines(currentImage, segmentedImage, segMsg);
  cv::Mat segmentedResizedOutImage;

  cv::resize(segmentedImage, segmentedResizedOutImage, cv::Size(), 0.4, 0.4);
  sensor_msgs::msg::CompressedImage::SharedPtr segmentedDebugImageMsg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", segmentedResizedOutImage)
      .toCompressedImageMsg();
  debugDetectedLinesPublisher->publish(*segmentedDebugImageMsg);

  sensor_msgs::msg::Image::SharedPtr segmentedImageMsg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", segmentedImage).toImageMsg();

  demo_msgs::msg::LSDandImage lsdMsg;
  lsdMsg.lines = segMsg;
  lsdMsg.image = *segmentedImageMsg;
  lineSegmentPublisher->publish(lsdMsg);
}

void ImageProcessingNode::updateCameraMatrix(const sensor_msgs::msg::CameraInfo & cameraInfo)
{
  m_cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m_cameraMatrix.at<double>(r, c) = cameraInfo.k.at(r * 3 + c);
    }
  }
}

void ImageProcessingNode::updateWarping(sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
  const int cols = cameraInfo->width;
  const int rows = cameraInfo->height;

  const int roi_x = static_cast<int>(cols * m_roiLeft);
  const int roi_y = static_cast<int>(rows * m_roiTop);
  const int roi_w = static_cast<int>(cols * (1.0F - m_roiLeft - m_roiRight));
  const int roi_h = static_cast<int>(rows * (1.0F - m_roiTop - m_roiBottom));

  cv::Mat whiteImage = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
  cv::Mat kernel =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(BEV_MASK_KERNEL_SIZE, BEV_MASK_KERNEL_SIZE));

  m_roi = cv::Rect(roi_x, roi_y, roi_w, roi_h);

  cv::Mat scaleMatrix =
    (cv::Mat_<double>(3, 3) << m_bevScaleFactor, 0, 0, 0, m_bevScaleFactor, 0, 0, 0, 1);

  cv::Mat baseTransform = (cv::Mat_<double>(3, 1) << cols / 2.0, rows, 1.0);

  double rollRad = m_cameraRollDeg * M_PI / 180.0;
  double pitchRad = m_cameraPitchDeg * M_PI / 180.0;
  double yawRad = m_cameraYawDeg * M_PI / 180.0;

  cv::Mat rotation = rotationMatrixFromEulerAngles(pitchRad, rollRad, yawRad);
  cv::Mat inverseCameraMatrix = m_cameraMatrix.inv();
  cv::Mat homography = m_cameraMatrix * rotation * inverseCameraMatrix;
  cv::Mat scaledHomography = scaleMatrix * homography;
  cv::Mat transformedLowerMiddle = scaledHomography * baseTransform;

  double translationX = (cols / 2.0) - transformedLowerMiddle.at<double>(0, 0) /
                                         transformedLowerMiddle.at<double>(2, 0);
  double translationY =
    rows - transformedLowerMiddle.at<double>(1, 0) / transformedLowerMiddle.at<double>(2, 0);

  cv::Mat translation = (cv::Mat_<double>(3, 3) << 1, 0, translationX, 0, 1, translationY, 0, 0, 1);

  m_homographyMatrix = translation * scaledHomography;

  cv::warpPerspective(whiteImage, m_mask, m_homographyMatrix, whiteImage.size(), cv::INTER_NEAREST);
  cv::erode(m_mask, m_mask, kernel, cv::Point(-1, -1), 3);
  m_mask = m_mask(m_roi);
}

void ImageProcessingNode::warpToBirdsEyeView(cv::Mat & image)
{
  cv::warpPerspective(
    image, image, m_homographyMatrix, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
    cv::Scalar(255, 255, 255));
  image = image(m_roi);
}

void ImageProcessingNode::detectLineSegments(cv::Mat & image)
{
  cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
  cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
  cv::adaptiveThreshold(
    image, image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, m_adaptiveThresholdblockSize,
    m_adaptiveThresholdC);
  cv::multiply(image, m_mask, image);
}

cv::Mat ImageProcessingNode::rotationMatrixFromEulerAngles(double roll, double pitch, double yaw)
{
  cv::Mat rotationX =
    (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll));

  cv::Mat rotationY =
    (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch));

  cv::Mat rotationZ =
    (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);

  return rotationZ * rotationY * rotationX;
}

void ImageProcessingNode::findLines(
  const cv::Mat & binaryImage, cv::Mat & segmentedImage, demo_msgs::msg::LineSegment & segMsg)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binaryImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> approxContours;
  approxContours.reserve(contours.size());
  int contoursSize = 0;
  for (const auto & contour : contours) {
    if (m_chainApproxEpsilon > 0.0F && m_applyContoursOptimization) {
      std::vector<cv::Point> approxContour;
      cv::approxPolyDP(contour, approxContour, m_chainApproxEpsilon, true);
      approxContours.push_back(approxContour);
      contoursSize += static_cast<int>(approxContour.size());
    } else {
      approxContours.push_back(contour);
      contoursSize += static_cast<int>(contour.size());
    }
  }

  std::vector<cv::Vec4f> lines;
  lines.reserve(contoursSize);
  for (const std::vector<cv::Point> & contour : approxContours) {
    if (contour.size() <= 1) {
      continue;
    }

    for (std::size_t i = 0; i < contour.size() - 1; ++i) {
      lines.emplace_back(contour[i].x, contour[i].y, contour[i + 1].x, contour[i + 1].y);
    }
    lines.emplace_back(contour.back().x, contour.back().y, contour.front().x, contour.front().y);
  }

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000, "Contours contains: %d points", contoursSize);

  segmentedImage = cv::Mat::zeros(binaryImage.rows, binaryImage.cols, binaryImage.type());
  for (const cv::Vec4f & line : lines) {
    cv::line(
      segmentedImage, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]),
      cv::Scalar(255, 0, 0));
  }

  demo_msgs::msg::LineSegment convertedLines = convertLines(lines);
  segMsg.detected_lines = convertedLines.detected_lines;
}

demo_msgs::msg::LineSegment ImageProcessingNode::convertLines(const std::vector<cv::Vec4f> & lines)
{
  auto segment = demo_msgs::msg::LineSegment();
  for (const cv::Vec4f & line : lines) {
    auto detectedLine = demo_msgs::msg::Line();
    detectedLine.x1 = line[0];
    detectedLine.y1 = line[1];
    detectedLine.x2 = line[2];
    detectedLine.y2 = line[3];
    segment.detected_lines.push_back(detectedLine);
  }
  return segment;
}

std::future<void> ImageProcessingNode::publishCompressedImage(
  const cv::Mat & image, const std_msgs::msg::Header & header,
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher)
{
  cv::Mat imageCopy = image.clone();
  return std::async(std::launch::async, [this, publisher, imageCopy, header]() {
    try {
      cv::Mat resizedOutImage;
      cv::resize(imageCopy, resizedOutImage, cv::Size(), 0.4, 0.4);
      sensor_msgs::msg::CompressedImage::SharedPtr debugImageMsg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", resizedOutImage).toCompressedImageMsg();
      debugImageMsg->header = header;
      publisher->publish(*debugImageMsg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Debug image publishing error: %s", e.what());
    }
  });
}

rcl_interfaces::msg::SetParametersResult ImageProcessingNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";

  for (const auto & param : parameters) {
    if (
      param.get_name() == "ROI_Top" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_roiTop = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "ROI_Bottom" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_roiBottom = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "ROI_Right" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_roiRight = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "ROI_Left" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_roiLeft = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "chain_approx_epsilon" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_chainApproxEpsilon = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "apply_contours_optimization" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_applyContoursOptimization = param.as_bool();
      result.successful = true;
    } else if (
      param.get_name() == "adaptive_threshold_block_size" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      m_adaptiveThresholdblockSize = static_cast<int>(param.as_int());
      result.successful = true;
    } else if (
      param.get_name() == "adaptive_threshold_c" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_adaptiveThresholdC = param.as_double();
      result.successful = true;
    } else if (
      param.get_name() == "roll" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_cameraRollDeg = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "pitch" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_cameraPitchDeg = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "yaw" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_cameraYawDeg = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "bev_scale_factor" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      m_bevScaleFactor = static_cast<float>(param.as_double());
      result.successful = true;
    } else if (
      param.get_name() == "print_timings" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      m_printTimings = param.as_bool();
      result.successful = true;
    }
  }

  m_updateWarping = true;
  return result;
}

}  // namespace image_processing

RCLCPP_COMPONENTS_REGISTER_NODE(image_processing::ImageProcessingNode)
