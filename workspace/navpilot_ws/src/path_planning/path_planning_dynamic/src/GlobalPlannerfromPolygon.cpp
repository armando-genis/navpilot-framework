#include "GlobalPlannerfromPolygon.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>
#include <nlohmann/json.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Eigen::Vector3f jsonPtToEigen(const nlohmann::json& j) {
    float x = 0.f, y = 0.f, z = 0.f;
    if (j.is_array()) {
        if (j.size() >= 1) x = j[0].get<float>();
        if (j.size() >= 2) y = j[1].get<float>();
        if (j.size() >= 3) z = j[2].get<float>();
    }
    return Eigen::Vector3f(x, y, z);
}

void polygonToLineSegments(const nlohmann::json& poly, std::vector<Eigen::Vector3f>& out) {
    if (!poly.is_array() || poly.size() < 2) return;
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t next = (i + 1) % poly.size();
        out.push_back(jsonPtToEigen(poly[i]));
        out.push_back(jsonPtToEigen(poly[next]));
    }
}

void polylineToLineSegments(const nlohmann::json& arr, std::vector<Eigen::Vector3f>& out) {
    if (!arr.is_array() || arr.size() < 2) return;
    for (size_t i = 0; i + 1 < arr.size(); ++i) {
        out.push_back(jsonPtToEigen(arr[i]));
        out.push_back(jsonPtToEigen(arr[i + 1]));
    }
}

void GlobalPlannerfromPolygon::setMapFile(const std::string& path) {
    map_file_path_ = path;
}

bool GlobalPlannerfromPolygon::loadAndPrintSummary() {
    if (map_file_path_.empty()) return false;
    std::ifstream f(map_file_path_);
    if (!f.is_open()) {
        std::cerr << "[HDMap] Could not open map file: " << map_file_path_ << std::endl;
        return false;
    }
    nlohmann::json root;
    try {
        f >> root;
    } catch (const std::exception& e) {
        std::cerr << "[HDMap] JSON parse error: " << e.what() << std::endl;
        return false;
    }
    int version = root.value("version", 0);
    if (version != 1) {
        std::cerr << "[HDMap] Unsupported HD-map file version " << version << " (expected 1)" << std::endl;
        return false;
    }

    size_t num_polygons = 0, poly_pts = 0;
    if (root.contains("polygons") && root["polygons"].is_array()) {
        const auto& polys = root["polygons"];
        num_polygons = polys.size();
        for (const auto& p : polys)
            if (p.is_array()) poly_pts += p.size();
    }

    waypoints_.clear();
    if (root.contains("centerline") && root["centerline"].is_array()) {
        const auto& cl = root["centerline"];
        for (size_t i = 0; i < cl.size(); ++i) {
            point_struct pt;
            if (cl[i].is_array() && cl[i].size() >= 2) {
                pt.x = cl[i].size() >= 1 ? cl[i][0].get<double>() : 0.0;
                pt.y = cl[i].size() >= 2 ? cl[i][1].get<double>() : 0.0;
            } else {
                continue;
            }
            pt.priority = 1;
            pt.lanelet_id = 0;
            pt.lane_sequence_id = static_cast<int>(i);
            if (i + 1 < cl.size() && cl[i + 1].is_array() && cl[i + 1].size() >= 2) {
                double nx = cl[i + 1][0].get<double>();
                double ny = cl[i + 1][1].get<double>();
                pt.heading = std::atan2(ny - pt.y, nx - pt.x);
            } else if (i > 0 && cl[i - 1].is_array() && cl[i - 1].size() >= 2) {
                double px = cl[i - 1][0].get<double>();
                double py = cl[i - 1][1].get<double>();
                pt.heading = std::atan2(pt.y - py, pt.x - px);
            } else {
                pt.heading = 0.0;
            }
            waypoints_.push_back(pt);
        }
    }
    size_t centerline_pts = waypoints_.size();

    size_t num_bike_lane_segments = 0;
    if (root.contains("bike_lane_segments") && root["bike_lane_segments"].is_array())
        num_bike_lane_segments = root["bike_lane_segments"].size();

    size_t bike_lane_active_pts = 0;
    if (root.contains("bike_lane_active") && root["bike_lane_active"].is_array())
        bike_lane_active_pts = root["bike_lane_active"].size();

    size_t num_crosswalks = 0;
    if (root.contains("crosswalks") && root["crosswalks"].is_array())
        num_crosswalks = root["crosswalks"].size();

    size_t num_parking_spaces = 0;
    if (root.contains("parking_spaces") && root["parking_spaces"].is_array())
        num_parking_spaces = root["parking_spaces"].size();

    size_t num_buildings = 0;
    if (root.contains("buildings") && root["buildings"].is_array())
        num_buildings = root["buildings"].size();

    loaded_ = true;
    std::cout << "[HDMap] Summary (frame: " << map_frame_ << "):"
              << " polygons=" << num_polygons << "(" << poly_pts << " pts)"
              << " centerline=" << centerline_pts << " pts"
              << " bike_lane_segments=" << num_bike_lane_segments
              << " bike_lane_active=" << bike_lane_active_pts << " pts"
              << " crosswalks=" << num_crosswalks
              << " parking_spaces=" << num_parking_spaces
              << " buildings=" << num_buildings
              << std::endl;
    return true;
}

std::vector<point_struct> GlobalPlannerfromPolygon::getAllAllWaypointsStruct() const {
    return waypoints_;
}

nav_msgs::msg::OccupancyGrid GlobalPlannerfromPolygon::getOccupancyGrid() const {
    return occupancy_grid_;
}


GlobalPlannerfromPolygon::GlobalPlannerfromPolygon()
{
}

GlobalPlannerfromPolygon::~GlobalPlannerfromPolygon()
{
}