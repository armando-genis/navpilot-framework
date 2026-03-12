#ifndef GLOBAL_PLANNER_FROM_POLYGON_HPP
#define GLOBAL_PLANNER_FROM_POLYGON_HPP

#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <array>

#include "GlobalPlanner.hpp"  // for point_struct


class GlobalPlannerfromPolygon
{

private:

    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    enum LineLayer {
        LAYER_POLYGONS = 0,
        LAYER_CENTERLINE,
        LAYER_BIKE_LANE,
        LAYER_CROSSWALKS,
        LAYER_PARKING,
        LAYER_BUILDINGS,
        NUM_LAYERS
    };

    std::string map_file_path_;
    std::string map_frame_ = "map";
    bool loaded_ = false;
    std::vector<point_struct> waypoints_;  // centerline as waypoints
    /** Polygon boundaries from JSON "polygons", each polygon as (x, y) with z=0. */
    std::vector<std::vector<std::array<double, 2>>> polygon_vertices_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    bool occupancy_grid_ready_ = false;

    double resolution_ = 0.2;
    int close_radius_ = 1;
    int close_iters_ = 1;
    int outside_value_ = 100;

    void generateOccupancyGrid();
    void worldToGrid(double wx, double wy, double min_x, double min_y, int& gx, int& gy) const;
    void fillPolygon(const std::vector<std::array<double, 2>>& points_xy, int width, int height,
                     double min_x, double min_y, std::vector<int8_t>& grid, int8_t value) const;
    void morphClose(std::vector<int8_t>& data, int width, int height, int radius, int iters) const;

public:
    GlobalPlannerfromPolygon();
    ~GlobalPlannerfromPolygon();

    /** Set map file path and frame from config (e.g. from config.yaml map_file). */
    void setMapFile(const std::string& path);

    /** Set occupancy grid parameters (same as GlobalPlanner). Call before loadAndPrintSummary(). */
    void setOccupancyGridParams(double resolution, int close_radius, int close_iters, int outside_value, const std::string& frame_id);

    /** Path to the HD map JSON file (e.g. /workspace/models/hdmap_export.json). */
    const std::string& getMapFilePath() const { return map_file_path_; }
    /** True if a map file has been set. */
    bool hasMapFile() const { return !map_file_path_.empty(); }
    /** True if map JSON was loaded and line geometry is uploaded (ready to draw). */
    bool isLoaded() const { return loaded_; }

    /**
     * Load the HD map JSON (same format as GLAutoLab hdMapIO.py), count elements,
     * build line geometry for all modules, and print a summary.
     * Returns true on success.
     */
    bool loadAndPrintSummary();

    /** Same interface as GlobalPlanner: waypoints from centerline (for path_planning integration). */
    std::vector<point_struct> getAllAllWaypointsStruct() const;
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;
    bool isOccupancyGridReady() const { return occupancy_grid_ready_; }

    /** Polygon line strips (z=0) as LINE_STRIP markers for RViz. */
    visualization_msgs::msg::MarkerArray getPolygonLineStripMarkers(const std::string& frame_id) const;
};

#endif