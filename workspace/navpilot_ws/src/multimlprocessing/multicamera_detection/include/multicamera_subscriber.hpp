#ifndef MULTICAMERA_DETECTION__MULTICAMERA_SUBSCRIBER_HPP_
#define MULTICAMERA_DETECTION__MULTICAMERA_SUBSCRIBER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "yolos/yolos.hpp"

#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"
#include "DataLoader.hpp"

namespace multicamera_detection
{

class MultiCameraSubscriber : public rclcpp::Node
{
public:
  explicit MultiCameraSubscriber();

private:
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id);

  int num_cameras_;
  std::string calib_dir_;
  std::string model_path_;
  bool use_gpu_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;

  std::unique_ptr<yolos::pose::YOLOPoseDetector> detector_;

  CalibrationLoader calib;

  // COCO skeleton connections (0-indexed)
  const std::vector<std::pair<int, int>> _SKELETON = {
      {0, 1}, {0, 2},           // nose -> eyes
      {1, 3}, {2, 4},           // eyes -> ears
      {5, 6},                   // shoulders
      {5, 7}, {7, 9},           // left arm
      {6, 8}, {8, 10},          // right arm
      {5, 11}, {6, 12},         // torso sides
      {11, 12},                 // hips
      {11, 13}, {13, 15},       // left leg
      {12, 14}, {14, 16},       // right leg
  };

  const std::vector<std::string> _KPT_NAMES = {
      "nose", "left_eye", "right_eye", "left_ear", "right_ear",
      "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
      "left_wrist", "right_wrist", "left_hip", "right_hip",
      "left_knee", "right_knee", "left_ankle", "right_ankle",
  };

  const cv::Scalar _KPT_COLOR = cv::Scalar(0, 255, 0);    // green keypoints
  const cv::Scalar _LIMB_COLOR = cv::Scalar(0, 165, 255);  // orange limbs
  const cv::Scalar _BOX_COLOR = cv::Scalar(255, 0, 0);    // blue bounding box
  const cv::Scalar _TEXT_COLOR = cv::Scalar(255, 255, 255);
  const float _CONF_THR = 0.3f;  // confidence threshold

};

}  // namespace multicamera_detection

#endif  // MULTICAMERA_DETECTION__MULTICAMERA_SUBSCRIBER_HPP_
