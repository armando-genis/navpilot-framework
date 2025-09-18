#ifndef WAYPOINTS_ROUTING_H
#define WAYPOINTS_ROUTING_H

#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/optional/optional_io.hpp>
#include <vector>
#include <cmath>
#include <iostream>

using namespace lanelet;
using namespace std;

class waypoints_routing : public rclcpp::Node
{
private:
    /* data */

    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    int start_lanelet_id = 0;
    int end_lanelet_id = 0;
    bool show_full_graph = true;
    double waypoint_interval = 3.0;  // Distance between waypoints in meters

    // Parameters
    std::string map_path_;
    visualization_msgs::msg::MarkerArray graph_waypoint_markers;
    visualization_msgs::msg::MarkerArray full_graph_markers;
    visualization_msgs::msg::MarkerArray neighbor_waypoint_markers;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr full_graph_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr neighbor_waypoints_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    void lanelet_routing_test(lanelet::LaneletMapPtr &map);
    void generateFullGraphVisualization(lanelet::LaneletMapPtr &map);
    void generateNeighborWaypoints(lanelet::LaneletMapPtr &map, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath);
    bool isSameDirection(const lanelet::ConstLanelet &lanelet1, const lanelet::ConstLanelet &lanelet2);
    bool isBeyondTarget(const lanelet::ConstLanelet &lanelet, const routing::LaneletPath &shortestPath);
    bool isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet);
    bool readParameters();
    void publishWaypoints();

    bool isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, lanelet::LaneletMapPtr &map);
    int countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, int current_path_index, lanelet::LaneletMapPtr &map);

    std::pair<double, double> getEndDirection(const std::vector<lanelet::ConstPoint3d> &points);
    std::pair<double, double> getStartDirection(const std::vector<lanelet::ConstPoint3d> &points);
    double calculatePathCurvature(const std::vector<lanelet::ConstPoint3d> &points);
    double calculatePathLength(const routing::LaneletPath &path);
    double calculateRemainingPathLength(const routing::LaneletPath &path, int start_index);

    double x_offset_;
    double y_offset_;

public:
    waypoints_routing(/* args */);
    ~waypoints_routing();
};

#endif // WAYPOINTS_ROUTING_H