#include "multicamera_subscriber.hpp"

namespace multicamera_processing
{

MultiCameraSubscriber::MultiCameraSubscriber() : Node("multicamera_processing_node")
{
  this->declare_parameter<int>("num_cameras", 3);
  this->declare_parameter<std::string>("calib_dir", "");
  this->declare_parameter<std::string>("lidar_topic", "");
  num_cameras_ = this->get_parameter("num_cameras").as_int();
  calib_dir_ = this->get_parameter("calib_dir").as_string();
  lidar_topic_ = this->get_parameter("lidar_topic").as_string();

  std::cout << "lidar_topic:" << lidar_topic_ <<std::endl;


  calib.load(calib_dir_);

  lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  auto qos = rclcpp::SensorDataQoS();

  // Subscribe to lidar topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_,
      qos,
      std::bind(&MultiCameraSubscriber::lidarCallback, this, std::placeholders::_1)
  );


  // Subscribe to camera topics
  for (int i = 0; i < num_cameras_; i++)
  {
    std::string topic = "/racecar/camera/camera_" + std::to_string(i) + "/image_raw";
    const int camera_id = i;

    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic,
        qos,
        [this, camera_id](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          imageCallback(msg, camera_id);
        }
    );

    subscribers_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
  }

  cv::namedWindow("MultiCamera", cv::WINDOW_NORMAL);
}

void MultiCameraSubscriber::imageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id)
{
  try
  {
    cv::Mat frame;

    if (msg->encoding == "mjpeg")
    {
      std::vector<uint8_t> buffer(msg->data.begin(), msg->data.end());
      frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }

    if (frame.empty())
      return;

    // Apply camera undistortion
    if (static_cast<size_t>(camera_id) < calib.camera_array.size())
    {
      auto cam = calib.camera_array[camera_id];

      if (cam)
      {
        cam->ensure_size(frame.cols, frame.rows);
        frame = cam->undistort(frame);
      }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(
        new pcl::PointCloud<pcl::PointXYZI>());

    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      *cloud_copy = *lidar_cloud_;
    }

    if (static_cast<size_t>(camera_id) < calib.extrinsics_array.size())
    {
      auto ext = calib.extrinsics_array[camera_id];

      if (ext && !cloud_copy->empty())
      {
        cv::Mat R = ext->get_R_opencv();
        cv::Mat t = ext->get_t_opencv();

        cv::Mat K = calib.camera_array[camera_id]->get_K();

        frame = projectLidarOnImage(frame, cloud_copy, R, t, K);
      }
    }

    cv::putText(
        frame,
        "Camera " + std::to_string(camera_id),
        {20, 40},
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        {0, 255, 0},
        2
    );

    cv::imshow("Camera " + std::to_string(camera_id), frame);
    cv::waitKey(1);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error decoding image: %s", e.what());
  }
}

void MultiCameraSubscriber::lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->data.empty())
    return;

  std::lock_guard<std::mutex> lock(lidar_mutex_);
  pcl::fromROSMsg(*msg, *lidar_cloud_);

}

cv::Mat MultiCameraSubscriber::projectLidarOnImage(
    const cv::Mat& img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& K)
{
  cv::Mat out = img.clone();

  if (!cloud || cloud->points.empty())
    return out;

  const double fx = K.at<double>(0,0);
  const double fy = K.at<double>(1,1);
  const double cx = K.at<double>(0,2);
  const double cy = K.at<double>(1,2);

  int h = out.rows;
  int w = out.cols;

  for (const auto& p : cloud->points)
  {
    cv::Mat pt = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);

    cv::Mat cam = R * pt + t;

    double x = cam.at<double>(0);
    double y = cam.at<double>(1);
    double z = cam.at<double>(2);

    if (z <= 0.1)
      continue;

    int u = static_cast<int>(fx * x / z + cx);
    int v = static_cast<int>(fy * y / z + cy);

    if (u < 0 || u >= w || v < 0 || v >= h)
      continue;

    float depth = std::min(static_cast<float>(z) / 20.0f, 1.0f);

    cv::Scalar color(
        255 * depth,
        255 * (1.0 - depth),
        255);

    cv::circle(out, {u,v}, 2, color, -1);
  }

  return out;
}

}  // namespace multicamera_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multicamera_processing::MultiCameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
