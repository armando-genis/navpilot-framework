#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

// path nav msgs
#include <nav_msgs/msg/path.hpp>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Custom msgs obstacles_information_msgs for Obstacle and ObstacleCollection
#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"

// Custom msgs traffic_information_msgs for RoadElements and RoadElementsCollection
#include "traffic_information_msgs/msg/road_elements.hpp"
#include "traffic_information_msgs/msg/road_elements_collection.hpp"

// tinyspline
#include <tinysplinecxx.h>

// STL containers
#include <set>

// STA collision checker
#include "sat_collision_checker.h"

// Car Data
#include "CarData.h"

// State
#include "State.h"

// Grid map
#include "Grid_map.h"

// Node
#include "Node.h"

// Global Planner
#include "GlobalPlanner.hpp"

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <utility>

// FlatNode and TreeFlat for the flat tree
struct FlatNode {
  State state;          // last state of this segment
  int   parent;         // index in `nodes` (-1 for root)
  double cost;          // cumulative path cost (fill as you like)
  double steer;         // steering used to reach this node (segment command)
  int    dir;           // direction used to reach this node (segment command)
  uint16_t depth;       // depth from root
};

struct TreeFlat {
  std::vector<FlatNode> nodes;   // flat storage of all nodes
  std::vector<int>      leaves;  // indices of leaf nodes in `nodes`
};

struct FlatNode_global_planner {
  State state;          // last state of this segment
  int   parent;         // index in `nodes` (-1 for root)
  double cost;          // cumulative path cost (fill as you like)
  int priority;         // priority of the waypoint
};

struct TreeFlat_global_planner {
  std::vector<FlatNode_global_planner> nodes;   // flat storage of all nodes
  std::vector<int>      leaves;  // indices of leaf nodes in `nodes`
};

static constexpr int    HEADING_BINS  = 64;
static constexpr double TWO_PI        = 6.28318530717958647692;
static constexpr double PI            = 3.14159265358979323846;


// Wrap to (-pi, pi]
static inline double wrapAngle(double a) {
    while (a >  PI) a -= TWO_PI;
    while (a <=-PI) a += TWO_PI;
    return a;
}

// Key for "same discrete state" at this resolution
struct LatticeKey {
    int gx, gy, hb;
    bool operator==(const LatticeKey& o) const noexcept {
        return gx==o.gx && gy==o.gy && hb==o.hb;
    }
};
struct LatticeKeyHash {
    size_t operator()(const LatticeKey& k) const noexcept {
        // 64-bit mix
        uint64_t x = (uint64_t)(uint32_t)k.gx;
        uint64_t y = (uint64_t)(uint32_t)k.gy;
        uint64_t h = (uint64_t)(uint32_t)k.hb;
        uint64_t z = (x * 0x9E3779B185EBCA87ULL) ^ (y << 6) ^ (y >> 2) ^ (h * 0xC2B2AE3D27D4EB4FULL);
        return (size_t)z;
    }
};

// Item in OPEN (min-heap by f)
struct PQItem {
    int idx;         // index in out.nodes
    double f_est;    // g + h_lb at push time
    double g_copy;   // g used to create this item (for staleness check)
    bool operator<(const PQItem& o) const noexcept {
        // std::priority_queue is max-heap, so invert
        return f_est > o.f_est;
    }
};

// Compute heading bin
static inline int heading_bin(double theta) {
    double t = wrapAngle(theta) + PI;                // [0, 2pi)
    double w = TWO_PI / (double)HEADING_BINS;
    int b = (int)std::floor(t / w);
    if (b < 0) b = 0;
    if (b >= HEADING_BINS) b = HEADING_BINS - 1;
    return b;
}

class path_planning : public rclcpp::Node
{
private:
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    // Continuous planning parameters
    double planning_frequency_ = 10.0;  // Hz
    double trajectory_time_step_ = 0.1; // seconds
    double max_trajectory_time_ = 3.0;  // seconds
    double target_velocity_ = 5.0;      // m/s
    double max_velocity_ = 10.0;        // m/s
    double max_acceleration_ = 2.0;     // m/s^2
    double max_curvature_ = 0.3;        // 1/m
    int num_lateral_offsets_ = 0;       // Number of lateral offset trajectories (0 = only center path)
    
    // Planning state
    bool continuous_planning_active_ = false;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    std::vector<State> current_trajectory_;
    State target_state_;
    double trajectory_progress_ = 0.0;

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    fop::SATCollisionChecker collision_checker; // Collision checker

    // Car Data
    CarData car_data_;
    double maxSteerAngle;
    double wheelBase;
    double axleToFront;
    double axleToBack;
    double width;

    // Grid Map
    std::shared_ptr<Grid_map> grid_map_;

    // State
    std::shared_ptr<State> car_state_;

    // global map and rescaled chunk 
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> global_map_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> rescaled_chunk_;

    // function to get the state (position) of the car
    void getCurrentRobotState();
    // function to get the closest waypoint to the car
    size_t closest_waypoint = 0;  // index of the closest waypoint to the car
    void compute_closest_waypoint();
    double getDistanceFromOdom(const point_struct& waypoint);
    
    // Continuous planning methods
    void startContinuousPlanning();
    void stopContinuousPlanning();
    void continuousPlanningCallback();
    std::vector<State> generateTrajectory(const State& start_state, const State& target_state, double time_horizon);
    std::vector<State> generateMultipleTrajectories(const State& start_state);
    std::vector<std::vector<State>> generateMultipleFullTrajectories(const State& start_state);
    State selectOptimalTrajectory(const std::vector<State>& trajectories);
    std::vector<State> selectOptimalFullTrajectory(const std::vector<std::vector<State>>& trajectories);
    void publishTrajectoryVisualization(const std::vector<State>& trajectory, const std::string& namespace_name);
    void publishAllTrajectoriesVisualization(const std::vector<std::vector<State>>& trajectories);
    double evaluateTrajectoryCost(const std::vector<State>& trajectory);
    std::vector<double> generateLateralOffsets();

    // =============================
    // global planner
    // =============================
    std::shared_ptr<GlobalPlanner> global_planner_;
    std::string map_path_;
    double x_offset_;
    double y_offset_;
    int start_lanelet_id_;
    int end_lanelet_id_;

    std::vector<point_struct> all_waypoints_from_global_planner_;  // waypoint with the central path and the neighbor lanelets
    visualization_msgs::msg::MarkerArray global_planner_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_planner_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_waypoint_marker_publisher_;
    void publishGlobalPlanner();
    void publish_closest_waypoint_marker();
    // =============================
    // map combination and convine with the map obstacles
    // =============================

    // subscription for the global map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_grid_map_sub_;
    void globalMap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    // subscription for the obstacle information
    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    // offset for the origin of the map chunk
    double forward_distance = 10.0;
    double forward_distance_square = 2.0; // this is for the white sqaure that ocloude the obstacles draw in the new map
    int chunk_size = 100;
    int chunk_radius = chunk_size / 2;
    double scale_factor = 1; // if the map resolution is 1.0 is a scale factor of 5 and if the map resolution is 0.2 the salce resultion shoudl be 1.
    void map_combination(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    cv::Mat toMat(const nav_msgs::msg::OccupancyGrid &map);
    cv::Mat rescaleChunk(const cv::Mat &chunk_mat, double scale_factor);
    // publisher for the occupancy grid 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_test_;
    
    // =============================
    // path prossecing and car dynamics
    // =============================

    // function to get the motion commands
    void motionCommands();
    std::vector<std::vector<double>> motionCommand; // [steering angle, direction]

    // variables for the path prossecing and car dynamics
    int pathLength; // (int): number of micro-steps per edge/primitive.
    double step_car;  // (m): distance advanced per micro-step.
    
    // tree structure parameters
    int tree_depth;        // Maximum depth of the tree (e.g., 3 levels)
    int branching_factor;  // Number of paths per node (e.g., 5)

    // white square parameters
    int square_size = 20; // Size of the square region in grid cells
    int half_square = square_size / 2;

    std::shared_ptr<planner::Node> current_node;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr real_trajectories_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr real_trajectories_pub_2;

    struct RelSample { double x, y, heading; };
    std::vector<std::vector<RelSample>> precomputed_rel_; // [cmd][step]
    void precomputeCommandSamples();

    // ---- scoring weights (tune as needed) ----
    double W_FORWARD = 0.8;   // maximize forward progress
    double W_LAT     = 0.3;   // penalize lateral offset from current heading axis
    double W_STEER   = 0.18;   // penalize steering effort (sum |steer| along chain)
    double W_HEAD    = 0.25;   // penalize heading error vs current heading (or goal)

    cv::Mat dist_m_; // distance matrix for the A* algorithm

    double SAFE_CLEAR = 0.8;    // meters: half vehicle width + margin
    double W_OBS      = 1.2;    // weight for clearance penalty
    double W_DSTEER   = 0.12;   // weight for smoothness (|Δsteer|)

    // Build chain indices root->leaf
    inline void build_chain_indices(const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const;

    // publish the best path from the flat tree
    void publishBestPathFromFlat(const TreeFlat& flat, int leaf_idx, int color_idx);

    // generate the trajectory based on the flat tree on the A* algorithm
    int generateTrajectoryTree_AStar_flat_map(const State& root_state, TreeFlat& out);

    void buildDistanceField();
    double clearanceMeters(int gx, int gy) const;


    // =============================
    // medium planner to get the path from the global planer
    // =============================

    int generateTajectoyTree_from_global_planer(const State& root_state, TreeFlat_global_planner& out);

    std::vector<point_struct> planner_waypoints_available;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planner_waypoints_available_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planner_waypoint_polygons_publisher_;
    
    // Frenet frame joined paths storage
    std::vector<point_struct> longitudinal_path_;
    std::vector<std::vector<point_struct>> lateral_paths_;
    
    // Continuous planning publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planned_trajectories_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimal_trajectory_publisher_;
    
    // Path joins publisher for Frenet frame trajectories
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_joins_publisher_;

    void publish_planner_waypoints_available();
    void publish_planner_waypoint_polygons();
    
    // Frenet frame path joining methods
    void createJoinedPaths();
    void publishJoinedPaths();
    std::vector<std::vector<point_struct>> groupWaypointsByLanelet();
    std::vector<point_struct> createLongitudinalPath();
    std::vector<std::vector<point_struct>> createLateralPaths();



public:
    path_planning();
    ~path_planning();
};

#endif

