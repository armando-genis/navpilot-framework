#include "multicamera_subscriber.hpp"

#include <iomanip>
#include <fstream>
#include <sstream>

namespace path_processing
{

namespace
{
void printMatLikePython(std::ostream& out, const cv::Mat& M, const char* name)
{
  out << name << ":\n";
  if (M.rows == 3 && M.cols == 3)
  {
    for (int r = 0; r < 3; r++)
    {
      out << (r == 0 ? "[[" : " [");
      for (int c = 0; c < 3; c++)
        out << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(r, c);
      out << (r == 2 ? "]]" : "]\n");
    }
    out << "\n\n";
  }
  else if (M.rows == 3 && M.cols == 1)
  {
    out << "[[ " << std::setw(11) << std::fixed << std::setprecision(8) << M.at<double>(0, 0) << "]\n";
    out << " [" << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(1, 0) << "]\n";
    out << " [" << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(2, 0) << "]]\n\n";
  }
}

void printMatScientific(std::ostream& out, const cv::Mat& M)
{
  out << std::scientific << std::setprecision(8);
  if (M.rows == 3 && M.cols == 3)
  {
    for (int r = 0; r < 3; r++)
    {
      out << (r == 0 ? "[[" : " [");
      for (int c = 0; c < 3; c++)
        out << std::setw(17) << M.at<double>(r, c);
      out << (r == 2 ? "]]" : "]\n");
    }
    out << "\n\n";
  }
  else if (M.rows == 4 && M.cols == 1)
  {
    for (int r = 0; r < 4; r++)
    {
      out << (r == 0 ? "[[ " : " [") << std::setw(12) << M.at<double>(r, 0);
      out << (r == 3 ? "]]" : " ]") << "\n";
    }
    out << "\n";
  }
  out << std::fixed;
}
}  // namespace

MultiCameraSubscriber::MultiCameraSubscriber() : Node("path_processing_node")
{
  this->declare_parameter<int>("num_cameras", 3);
  this->declare_parameter<std::string>("calib_dir", "");
  this->declare_parameter<std::string>("waypoints_file_path", "");
  num_cameras_ = this->get_parameter("num_cameras").as_int();
  calib_dir_ = this->get_parameter("calib_dir").as_string();
  waypoints_file_path_ = this->get_parameter("waypoints_file_path").as_string();

  calib.load(calib_dir_);
  readWaypoints();

  if (static_cast<size_t>(0) < calib.camera_array.size() && calib.camera_array[0])
  {
    auto* cam = calib.camera_array[0].get();
    std::cout << "\nIntrinsics: LOADED\n\n";
    cv::Size sz = cam->get_frame_size();
    std::cout << "Frame size (w x h): (" << sz.width << ", " << sz.height << ")\n\n";
    std::cout << "Intrinsics (K):\n";
    printMatScientific(std::cout, cam->get_K());
    std::cout << "Distortion (D):\n";
    printMatScientific(std::cout, cam->get_D());
  }

  if (static_cast<size_t>(0) < calib.extrinsics_array.size() && calib.extrinsics_array[0])
  {
    auto* ext = calib.extrinsics_array[0].get();
    std::cout << "Extrinsics (Lidar \u2192 Camera): LOADED\n\n";
    printMatLikePython(std::cout, ext->get_R_opencv(), "R (opencv_frame)");
    printMatLikePython(std::cout, ext->get_t_opencv(), "t (opencv_frame)");
    printMatLikePython(std::cout, ext->get_R_robot(), "R (robot_frame)");
    printMatLikePython(std::cout, ext->get_t_robot(), "t (robot_frame)");
    const auto& lr = ext->get_lidar_rotation();
    std::cout << "Lidar rotation: {'axis_x': " << std::fixed << std::setprecision(8) << lr.axis_x
              << ", 'axis_y': " << lr.axis_y << ", 'axis_z': " << lr.axis_z << "}\n" << std::endl;
  }

  auto qos = rclcpp::SensorDataQoS();

  // Subscribe to camera topics
  for (int i = 0; i < num_cameras_; i++)
  {
    std::string topic = "/racecar/camera/camera_" + std::to_string(i) + "/image_raw_sync";
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

void MultiCameraSubscriber::readWaypoints()
{
  if (waypoints_file_path_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Waypoints file path is empty, skipping load");
    return;
  }

  std::ifstream file(waypoints_file_path_);
  if (!file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", waypoints_file_path_.c_str());
    return;
  }

  std::string line;
  // Skip the header line
  std::getline(file, line);

  waypoints_.clear();
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string value;
    Eigen::VectorXd waypoint(6);  // x, y, z, yaw, velocity, change_flag

    for (int i = 0; i < 6; ++i)
    {
      std::getline(ss, value, ',');
      waypoint(i) = std::stod(value);
    }

    waypoints_.push_back(waypoint);
  }

  file.close();
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), waypoints_file_path_.c_str());
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

    // Undistort image
    if (static_cast<size_t>(camera_id) < calib.camera_array.size())
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

}  // namespace path_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_processing::MultiCameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
