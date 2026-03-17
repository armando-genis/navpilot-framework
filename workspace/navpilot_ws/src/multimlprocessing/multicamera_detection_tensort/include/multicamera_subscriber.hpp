#ifndef MULTICAMERA_DETECTION_TENSORT__MULTICAMERA_SUBSCRIBER_HPP_
#define MULTICAMERA_DETECTION_TENSORT__MULTICAMERA_SUBSCRIBER_HPP_

#include <memory>
#include <thread>
#include <condition_variable>
#include <optional>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include "yolov8.h"                    // provides: YoloV8, YoloV8Config, Object
#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"
#include "DataLoader.hpp"
#include "LidarCameraTree.hpp"


namespace multicamera_detection_tensort
{

class MultiCameraSubscriber : public rclcpp::Node
{
public:
  explicit MultiCameraSubscriber();
  ~MultiCameraSubscriber() override;

private:
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id);
  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void inferenceLoop();

  cv::Mat projectLidarOnImage(
      const cv::Mat& img,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      const cv::Mat& R, const cv::Mat& t, const cv::Mat& K);

  // det.kps layout: [x0, y0, conf0, x1, y1, conf1, ...]  (17 COCO keypoints)
  std::optional<float> computePersonOrientationFromKpts(
      const Object& det,
      int camera_id, float scale, int pad_x, int pad_y);

  int         num_cameras_;
  std::string calib_dir_;
  std::string lidar_topic_;
  std::string model_path_;
  bool        use_gpu_;
  bool        enable_yaw_{false};

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  // publisher and ID counter:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  int marker_id_{0};   // cycles 0–999

  std::unique_ptr<YoloV8> yolo_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
  std::mutex   lidar_mutex_;
  LidarCameraTree lidar_tree_;
  std::atomic<bool> tree_dirty_{false};
  CalibrationLoader calib;

  struct PendingFrame {
    cv::Mat                              image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    int                                  camera_id;
  };

  std::mutex              queue_mutex_;
  std::condition_variable queue_cv_;
  std::unique_ptr<PendingFrame> pending_frame_;
  bool        running_ = true;
  std::thread infer_thread_;

  const float _CONF_THR = 0.3f;
};

}  // namespace multicamera_detection_tensort
#endif