#pragma once
#include <future>
#include <memory>
#include <vector>

#include "demo_msgs/msg/line_segment.hpp"
#include "demo_msgs/msg/ls_dand_image.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace image_processing {

class ImageProcessingNode : public rclcpp::Node
{
public:
  explicit ImageProcessingNode(const rclcpp::NodeOptions & options);
  ~ImageProcessingNode() = default;

private:
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rclcpp::Publisher<demo_msgs::msg::LSDandImage>::SharedPtr lineSegmentPublisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugWarpedImagePublisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugDetectedLinesPublisher;

  cv::Mat m_cameraMatrix;
  cv::Mat m_mask;
  cv::Rect m_roi;
  cv::Mat m_homographyMatrix;

  float m_roiTop = 0.0F;
  float m_roiBottom = 0.0F;
  float m_roiRight = 0.0F;
  float m_roiLeft = 0.0F;
  float m_chainApproxEpsilon = 0.0F;
  bool m_applyContoursOptimization = true;
  int m_adaptiveThresholdblockSize = 51;
  double m_adaptiveThresholdC = -20.0;
  float m_cameraRollDeg = 0.0F;
  float m_cameraPitchDeg = 90.0F;
  float m_cameraYawDeg = 0.0F;
  float m_bevScaleFactor = 0.1F;

  sensor_msgs::msg::CameraInfo m_lastCameraInfo;

  bool m_printTimings = false;

  // bool to mark whether all associated variables for the BEV warping need to be updated. This is
  // set to true when the camera parameters are changed.
  bool m_updateWarping = true;

  using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_cameraImageSubscriber;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> m_cameraInfoSubscriber;
  std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>> m_cameraSynchronizer;

  void cameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);

  bool cameraInfoContentChanged(const sensor_msgs::msg::CameraInfo & info) const;

  void processImage(sensor_msgs::msg::Image::ConstSharedPtr cameraImageMsg);
  void updateCameraMatrix(const sensor_msgs::msg::CameraInfo & cameraInfo);
  void updateWarping(sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);
  void warpToBirdsEyeView(cv::Mat & image);
  void detectLineSegments(cv::Mat & image);

  static cv::Mat rotationMatrixFromEulerAngles(double roll, double pitch, double yaw);
  void findLines(
    const cv::Mat & binaryImage, cv::Mat & segmentedImage, demo_msgs::msg::LineSegment & segMsg);
  static demo_msgs::msg::LineSegment convertLines(const std::vector<cv::Vec4f> & lines);
  std::future<void> publishCompressedImage(
    const cv::Mat & image, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher);

  /**
   * Callback for rqt reconfigure
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace image_processing
