#ifndef MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
#define MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"
#include "DataLoader.hpp"

namespace multicamera_processing
{

class MultiCameraSubscriber : public rclcpp::Node
{
public:
  explicit MultiCameraSubscriber();

private:
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id);

  int num_cameras_;
  std::string calib_dir_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;

  CalibrationLoader calib;

};

}  // namespace multicamera_processing

#endif  // MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
