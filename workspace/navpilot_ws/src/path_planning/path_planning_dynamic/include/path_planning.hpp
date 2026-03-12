#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

// Global Planner from Polygon
#include "GlobalPlannerfromPolygon.hpp"

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
    /** @return true if transform was available and state was updated, false otherwise */
    bool getCurrentRobotState();

    // =============================
    // global planner
    // =============================
    std::string type_map_;  // "lanelet2" or "polygon"
    std::shared_ptr<GlobalPlanner> global_planner_;
    std::shared_ptr<GlobalPlannerfromPolygon> global_planner_polygon_;
    std::string map_path_;
    std::string polygon_path_;
    double x_offset_;
    double y_offset_;
    int start_lanelet_id_;
    int end_lanelet_id_;

    std::vector<point_struct> all_waypoints_from_global_planner_;  // waypoint with the central path and the neighbor lanelets
    visualization_msgs::msg::MarkerArray global_planner_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_planner_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_line_strips_publisher_;
    void publishGlobalPlanner();
    void publishGlobalPlannerOccupancyGrid();
    void publishPolygonLineStrips();
    // =============================
    // map combination and convine with the map obstacles
    // =============================

    // subscription for the obstacle information
    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::ConstSharedPtr msg);
    // offset for the origin of the map chunk
    double forward_distance = 7.0;
    int chunk_size = 100;
    int chunk_radius = chunk_size / 2;
    double scale_factor = 1; // if the map resolution is 1.0 is a scale factor of 5 and if the map resolution is 0.2 the salce resultion shoudl be 1.
    void map_combination(const obstacles_information_msgs::msg::ObstacleCollection::ConstSharedPtr msg);
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

    // white square parameters (meters): region cleared around car to avoid self-obstacles
    double white_square_length_m_ = 3.0;  // along vehicle (forward/back)
    double white_square_width_m_ = 3.0;   // lateral (left/right)
    double forward_distance_square = 0.0;   // offset of white square center along heading (m)


    std::shared_ptr<planner::Node> current_node;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr real_trajectories_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr real_trajectories_pub_2;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr all_paths_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sdv_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr car_footprint_pub_;
    rclcpp::TimerBase::SharedPtr car_footprint_timer_;

    struct RelSample { double x, y, heading; };
    std::vector<std::vector<RelSample>> precomputed_rel_; // [cmd][step]
    void precomputeCommandSamples();

    // ---- scoring weights (tune as needed) ----
    double W_FORWARD = 1.0;   // maximize forward progress
    double W_LAT     = 0.3;   // penalize lateral offset from current heading axis
    double W_STEER   = 0.1;   // penalize steering effort (sum |steer| along chain)
    double W_HEAD    = 0.2;   // penalize heading error vs current heading (or goal)

    cv::Mat dist_m_; // distance matrix for the A* algorithm

    double SAFE_CLEAR = 0.8;    // meters: half vehicle width + margin
    double W_OBS      = 1.2;    // weight for clearance penalty
    double W_DSTEER   = 0.05;   // weight for smoothness (|Δsteer|)

    // Build chain indices root->leaf
    inline void build_chain_indices(const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const;

    // publish the best path from the flat tree
    void publishBestPathFromFlat(const TreeFlat& flat, int leaf_idx, int color_idx);
    void publishAllPathsFromFlat(const TreeFlat& flat);
    void publishTrajectoryPath(const TreeFlat& flat, int leaf_idx);
    void publishCarFootprintPolygon();

    // generate the trajectory based on the flat tree on the A* algorithm
    int generateTrajectoryTree_AStar_flat_map(const State& root_state, TreeFlat& out);
    int generateTrajectoryTree_AStar_flat_map_with_waypoints(const State& root_state, TreeFlat& out);

    void buildDistanceField();
    double clearanceMeters(int gx, int gy) const;

    // --- waypoint attraction (priority-aware) ---
    cv::Mat dist_wp1_m_;   // meters to priority-1 path
    cv::Mat dist_wp2_m_;   // meters to priority-2 path
    bool has_wp1_ = false;
    bool has_wp2_ = false;

    double W_WP1 = 1.2;    // weight for distance to prio-1 path (balanced for straight/curved)
    double W_WP2 = 0.6;    // weight for distance to prio-2 path (balanced for straight/curved)
    double WP_STROKE_RADIUS_CELLS = 2.0; // thickness when rasterizing lines

    void buildWaypointDistanceFields();  // builds dist_wp1_m_ / dist_wp2_m_

    // Occupancy grid parameters
    double global_planner_resolution_;
    int global_planner_close_radius_ = 1;
    int global_planner_close_iters_ = 1;
    int global_planner_outside_value_;
    std::string global_planner_frame_id_;
    std::string global_planner_occupancy_output_topic_;
    nav_msgs::msg::OccupancyGrid global_planner_occupancy_grid_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_planner_occupancy_grid_publisher_;

public:
    path_planning();
    ~path_planning();
};

#endif
