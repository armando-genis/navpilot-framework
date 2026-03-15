#ifndef PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
#define PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"
#include "DataLoader.hpp"

namespace path_processing
{

class MultiCameraSubscriber : public rclcpp::Node
{
public:
  explicit MultiCameraSubscriber();

private:
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id);
  void readWaypoints();

  int num_cameras_;
  std::string calib_dir_;
  std::string waypoints_file_path_;
  std::ofstream ofs_;
  std::vector<Eigen::VectorXd> waypoints_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;

  CalibrationLoader calib;

};

}  // namespace path_processing

#endif  // PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
