#ifndef MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
#define MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


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
  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  int num_cameras_;
  std::string calib_dir_;
  std::string lidar_topic_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
  std::mutex lidar_mutex_;

  cv::Mat projectLidarOnImage(
      const cv::Mat& img,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      const cv::Mat& R,
      const cv::Mat& t,
      const cv::Mat& K);

  CalibrationLoader calib;

};

}  // namespace multicamera_processing

#endif  // MULTICAMERA_PROCESSING__MULTICAMERA_SUBSCRIBER_HPP_
