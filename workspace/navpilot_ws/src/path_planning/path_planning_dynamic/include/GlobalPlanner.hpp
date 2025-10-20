#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/optional/optional_io.hpp>
#include <vector>
#include <cmath>
#include <iostream>

using namespace lanelet;
using namespace std;

struct point_struct { double x, y, heading; 
                    int priority, lanelet_id, lane_sequence_id;};

class GlobalPlanner
{
private:

    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    //global planner parameters
    double waypoint_interval = 0.5;
    int start_lanelet_id_ = 0;
    int end_lanelet_id_ = 0;
    double x_offset_ = 0.0;
    double y_offset_ = 0.0;
    std::string map_path_ = "";

    void map_routing(lanelet::LaneletMapPtr &map);

    std::vector<std::vector<point_struct>> neighbor_points_;  // point if the neighbor lanelet
    std::vector<point_struct> all_waypoints_;  // point if the neighbor lanelet

    // get paths and neighbors
    void generateNeighborWaypoints(lanelet::LaneletMapPtr &map, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath);
    bool isBeyondTarget(const lanelet::ConstLanelet &lanelet, const routing::LaneletPath &shortestPath);
    bool isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet);
    bool isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, lanelet::LaneletMapPtr &map);
    int countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, int current_path_index, lanelet::LaneletMapPtr &map);
    std::pair<double, double> getEndDirection(const std::vector<lanelet::ConstPoint3d> &points);
    std::pair<double, double> getStartDirection(const std::vector<lanelet::ConstPoint3d> &points);
    double calculatePathLength(const routing::LaneletPath &path);
    double calculateRemainingPathLength(const routing::LaneletPath &path, int start_index);
    std::vector<point_struct> getAllWaypointsStruct() const;

    // Occupancy grid helper functions
    void generateOccupancyGrid(lanelet::LaneletMapPtr &t_map);
    void worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const;
    void drawLine(int x0, int y0, int x1, int y1, int width, int height, std::vector<int8_t> &data, int8_t value) const;
    void morphClose(std::vector<int8_t> &data, int width, int height, int radius, int iters) const;
    void fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points, int width, int height, 
                            double min_x, double min_y, std::vector<int8_t> &grid, int8_t value) const;


    // Occupancy grid parameters
    double resolution_;
    int close_radius_ = 1;
    int close_iters_ = 1;
    int outside_value_;
    std::string frame_id_;

      // Occupancy grid data
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    bool occupancy_grid_ready_;

public:
    GlobalPlanner(double x_offset, double y_offset, std::string map_path, int start_lanelet_id, int end_lanelet_id, double resolution, int close_radius, int close_iters, int outside_value, std::string frame_id);
    ~GlobalPlanner();
    std::vector<point_struct> getAllAllWaypointsStruct();

    nav_msgs::msg::OccupancyGrid getOccupancyGrid();
    bool isOccupancyGridReady();
};

#endif // GLOBAL_PLANNER_HPP