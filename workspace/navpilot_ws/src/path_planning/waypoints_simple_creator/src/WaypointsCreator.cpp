
#include <rclcpp/rclcpp.hpp>

// ROS
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// C++
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <iomanip>


using namespace std;

class WaypointsCreator : public rclcpp::Node
{
private:
  // variables 
  double current_pose_x_= 0;
  double current_pose_y_= 0;
  double current_pose_z_ = 0;

  double current_velocity_ = 0;


  double interval_ = 2;
  bool first_data_received_ = false;

  std::string file_path_ = "wp.csv";
  std::ofstream ofs_;

  nav_msgs::msg::Odometry current_pose_;
  nav_msgs::msg::Odometry previous_pose_;

  std::string global_frame_id_ = "map";
  std::string robot_frame_id_ = "velodyne";

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  bool getCurrentRobotPose();

  double computeDistance(const nav_msgs::msg::Odometry& odom_pos1 , const nav_msgs::msg::Odometry& odom_pos2);
  void writePoseToFile(const nav_msgs::msg::Odometry& odom_pos, bool change_flag);
  double yaw_callback(const nav_msgs::msg::Odometry& odom_pos) const;
  void waypointsPath();
  void writeHeaderToFile();
  void pub_callback();

  void displayMarker(const nav_msgs::msg::Odometry& odom_pos);
  void displayMarkerinfo(const nav_msgs::msg::Odometry& odom_pos);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_saver_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_info_pub_;


public:
  WaypointsCreator(/* args */);
  ~WaypointsCreator();
};

WaypointsCreator::WaypointsCreator(/* args */) : Node("waypoints_creator_node"),
  tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{
  this->declare_parameter("global_frame_id", global_frame_id_);
  this->declare_parameter("robot_frame_id", robot_frame_id_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("robot_frame_id", robot_frame_id_);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&WaypointsCreator::pub_callback, this));
  waypoint_saver_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", 10);
  waypoint_info_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_info", 10);


  writeHeaderToFile();
  
  RCLCPP_INFO(this->get_logger(), "\033[1;32m----> waypoints_creator_node initialized.\033[0m");


}

WaypointsCreator::~WaypointsCreator()
{
  ofs_.close(); // Close the file
}

double WaypointsCreator::computeDistance(const nav_msgs::msg::Odometry& odom_pos1 , const nav_msgs::msg::Odometry& odom_pos2){
    double x1 = odom_pos1.pose.pose.position.x;
    double y1 = odom_pos1.pose.pose.position.y;
    double x2 = odom_pos2.pose.pose.position.x;
    double y2 = odom_pos2.pose.pose.position.y;
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool WaypointsCreator::getCurrentRobotPose()
{
  geometry_msgs::msg::Transform pose_tf;
  try
  {
    pose_tf = tf2_buffer.lookupTransform(global_frame_id_, robot_frame_id_, tf2::TimePointZero).transform;
    current_pose_.header.frame_id = global_frame_id_;
    current_pose_.header.stamp = this->now();
    current_pose_.pose.pose.position.x = pose_tf.translation.x;
    current_pose_.pose.pose.position.y = pose_tf.translation.y;
    current_pose_.pose.pose.position.z = pose_tf.translation.z - 2.10;
    current_pose_.pose.pose.orientation = pose_tf.rotation;

    if (first_data_received_) {
      double dt = (this->now() - previous_pose_.header.stamp).seconds();
      if (dt > 1e-6) {
        double dist = computeDistance(current_pose_, previous_pose_);
        current_pose_.twist.twist.linear.x = dist / dt;
      }
    } else {
      current_pose_.twist.twist.linear.x = 0.0;
    }
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Transform unavailable (%s -> %s): %s. Skipping until TF is available.",
        global_frame_id_.c_str(), robot_frame_id_.c_str(), ex.what());
    return false;
  }
}

void WaypointsCreator::pub_callback()
{
  if (!getCurrentRobotPose()) {
    return;
  }
  if (!first_data_received_) {
    first_data_received_ = true;
    RCLCPP_INFO(this->get_logger(), "First data received (TF)!");
    writePoseToFile(current_pose_, false);
    displayMarker(current_pose_);
    displayMarkerinfo(current_pose_);
    previous_pose_ = current_pose_;
  }
  waypointsPath();
}

void WaypointsCreator::writeHeaderToFile() {
  ofs_.open(file_path_, std::ios::out);  // Open the file here
  if (!ofs_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", file_path_.c_str());
    return;
  }
  ofs_ << "x,y,z,yaw,velocity,change_flag" << std::endl;
}

void WaypointsCreator::writePoseToFile(const nav_msgs::msg::Odometry& odom_pos, bool change_flag){
  if (ofs_) {
    double current_pose_yaw_ = yaw_callback(odom_pos);
    ofs_ << std::fixed << std::setprecision(4) 
        << odom_pos.pose.pose.position.x << "," 
        << odom_pos.pose.pose.position.y << ","
        << odom_pos.pose.pose.position.z << "," 
        << current_pose_yaw_ << "," 
        << odom_pos.twist.twist.linear.x << ","
        << change_flag 
        << std::endl;
  }
  RCLCPP_INFO(this->get_logger(), "Enter to writePoseToFile");
}


double WaypointsCreator::yaw_callback(const nav_msgs::msg::Odometry& odom_pos) const {
    // Convert ROS2 Quaternion message to tf2 Quaternion
    tf2::Quaternion q(
        odom_pos.pose.pose.orientation.x,
        odom_pos.pose.pose.orientation.y,
        odom_pos.pose.pose.orientation.z,
        odom_pos.pose.pose.orientation.w);

    // Extract yaw from the quaternion
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}



void WaypointsCreator::waypointsPath(){
  if (first_data_received_){
    double distance = computeDistance(current_pose_, previous_pose_);
      if (distance > interval_) {
      writePoseToFile(current_pose_, false);
      displayMarker(current_pose_);
      displayMarkerinfo(current_pose_);
      previous_pose_ = current_pose_;
      RCLCPP_INFO(this->get_logger(), "New waypoint added");
    }
  }
}


void WaypointsCreator::displayMarkerinfo(const nav_msgs::msg::Odometry& odom_pos){
  static int id = 3000;
  static visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::Pose current_pose_from_odom = odom_pos.pose.pose;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "waypoints";
  marker.id = id;

  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.ns = "waypoints_info";
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = current_pose_from_odom;

  std::ostringstream oss;
  // oss << std::fixed << std::setprecision(2) << odom_pos.twist.twist.linear.x << " km/h";
  oss << std::fixed << std::setprecision(3) << current_pose_from_odom.orientation.x << " & " << current_pose_from_odom.orientation.y;

  marker.text = oss.str();
  marker_array.markers.push_back(marker);
  waypoint_info_pub_->publish(marker_array);

  id++;

}


void WaypointsCreator::displayMarker(const nav_msgs::msg::Odometry& odom_pos){
  static int id = 0;
  static visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::Pose current_pose_from_odom = odom_pos.pose.pose;


  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  // marker.frame_locked = true;
  marker.ns = "waypoints";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = current_pose_from_odom;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);

  waypoint_saver_pub_->publish(marker_array);
  id++;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
