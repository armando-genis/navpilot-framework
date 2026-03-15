#ifndef PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
#define PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  bool getCurrentRobotPose();

  std::vector<cv::Mat> frame_cache_;  // size = num_cameras_
  std::vector<bool>    frame_ready_;  // size = num_cameras_

  // Add to private methods:
  void bakePathOnImages(std::vector<cv::Mat>& images, float path_width = 1.5f);

  int num_cameras_;
  std::string calib_dir_;
  std::string waypoints_file_path_;
  std::ofstream ofs_;
  std::vector<Eigen::VectorXd> waypoints_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;

  std::string global_frame_id_;
  std::string robot_frame_id_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  nav_msgs::msg::Odometry current_pose_;

  CalibrationLoader calib;

};

}  // namespace path_processing

#endif  // PATH_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
