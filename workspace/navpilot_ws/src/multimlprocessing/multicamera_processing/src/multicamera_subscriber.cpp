#include "multicamera_subscriber.hpp"

namespace multicamera_processing
{

MultiCameraSubscriber::MultiCameraSubscriber() : Node("multicamera_processing_node")
{
  this->declare_parameter<int>("num_cameras", 3);
  this->declare_parameter<std::string>("calib_dir", "");

  num_cameras_ = this->get_parameter("num_cameras").as_int();
  calib_dir_ = this->get_parameter("calib_dir").as_string();

  calib.load(calib_dir_);

  auto qos = rclcpp::SensorDataQoS();

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

    // ---------------------------------
    // Apply camera undistortion
    // ---------------------------------
    if (camera_id < calib.camera_array.size())
    {
      auto cam = calib.camera_array[camera_id];

      if (cam)
      {
        cam->ensure_size(frame.cols, frame.rows);
        frame = cam->undistort(frame);
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

}  // namespace multicamera_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multicamera_processing::MultiCameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
