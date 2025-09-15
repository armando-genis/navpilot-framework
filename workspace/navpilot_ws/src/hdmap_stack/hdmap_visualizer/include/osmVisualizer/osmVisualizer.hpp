
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>

#include <lanelet2_core/geometry/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <boost/optional/optional_io.hpp>

// custon messages
// RoadElements
#include "traffic_information_msgs/msg/road_elements.hpp"
// for RoadElementsCollection
#include "traffic_information_msgs/msg/road_elements_collection.hpp"

// we want assert statements to work in release mode
#undef NDEBUG

using namespace std::chrono_literals;
using namespace lanelet;

class OsmVisualizer : public rclcpp::Node
{
public:
  OsmVisualizer();

private:
  void timer_callback();
  bool readParameters();
  void writeToFile(const std_msgs::msg::Float64MultiArray &multi_array);
  void fill_marker(lanelet::LaneletMapPtr &t_map);
  void fill_array(lanelet::LaneletMapPtr &t_map);
  void fill_array_with_left_right(lanelet::LaneletMapPtr &t_map);
  void generateOccupancyGrid(lanelet::LaneletMapPtr &t_map);
  void publishOccupancyGrid();
  double getDistance(const lanelet::ConstLanelet &ll, size_t i);
  
  // Occupancy grid helper functions
  void worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const;
  void drawLine(int x0, int y0, int x1, int y1, int width, int height, std::vector<int8_t> &data, int8_t value) const;
  void morphClose(std::vector<int8_t> &data, int width, int height, int radius, int iters) const;
  void fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points, int width, int height, 
                         double min_x, double min_y, std::vector<int8_t> &grid, int8_t value) const;
  void floodFill(std::vector<int8_t> &grid, int width, int height, int start_x, int start_y, int8_t fill_value) const;
  void floodFillInterior(std::vector<int8_t> &grid, int width, int height, int start_x, int start_y, int8_t fill_value) const;
  bool isPointInPolygon(int px, int py, const std::vector<std::pair<int, int>>& polygon) const;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygon_publisher_;
  rclcpp::Publisher<traffic_information_msgs::msg::RoadElementsCollection>::SharedPtr road_elements_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;

  std_msgs::msg::Float64MultiArray m_array;
  visualization_msgs::msg::MarkerArray m_marker_array;

  // msgs for the crosswalks and road information
  polygon_msgs::msg::Polygon2DCollection crosswalk_polygons;
  traffic_information_msgs::msg::RoadElementsCollection road_elements;

  // colors for the terminal
  std::string green = "\033[1;32m";
  std::string red = "\033[1;31m";
  std::string blue = "\033[1;34m";
  std::string yellow = "\033[1;33m";
  std::string purple = "\033[1;35m";
  std::string reset = "\033[0m";

  bool m_first{true};
  bool m_second{true};
  bool m_third{true};

  // params
  std::string map_path_;
  bool enable_inc_path_points_;
  double interval_;

  double x_offset_;
  double y_offset_;
  
  // Occupancy grid parameters
  double resolution_;
  int close_radius_ = 1;
  int close_iters_ = 1;
  int outside_value_;
  std::string frame_id_;
  std::string occupancy_output_topic_;
  
  // Occupancy grid data
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  bool occupancy_grid_ready_;
  bool occupancy_dirty_;
};