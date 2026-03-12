#include "GlobalPlannerfromPolygon.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <limits>

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

void GlobalPlannerfromPolygon::setOccupancyGridParams(double resolution, int close_radius, int close_iters, int outside_value, const std::string& frame_id) {
    resolution_ = resolution;
    close_radius_ = close_radius;
    close_iters_ = close_iters;
    outside_value_ = outside_value;
    map_frame_ = frame_id;
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

    polygon_vertices_.clear();
    size_t num_polygons = 0, poly_pts = 0;
    if (root.contains("polygons") && root["polygons"].is_array()) {
        const auto& polys = root["polygons"];
        num_polygons = polys.size();
        for (const auto& p : polys) {
            if (!p.is_array() || p.size() < 2) continue;
            std::vector<std::array<double, 2>> poly_xy;
            poly_xy.reserve(p.size());
            for (size_t k = 0; k < p.size(); ++k) {
                if (!p[k].is_array() || p[k].size() < 2) continue;
                double x = p[k].size() >= 1 ? p[k][0].get<double>() : 0.0;
                double y = p[k].size() >= 2 ? p[k][1].get<double>() : 0.0;
                poly_xy.push_back({{x, y}});
            }
            if (poly_xy.size() >= 2) {
                polygon_vertices_.push_back(std::move(poly_xy));
                poly_pts += polygon_vertices_.back().size();
            }
        }
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
    generateOccupancyGrid();
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

void GlobalPlannerfromPolygon::worldToGrid(double wx, double wy, double min_x, double min_y, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - min_x) / resolution_));
    gy = static_cast<int>(std::floor((wy - min_y) / resolution_));
}

void GlobalPlannerfromPolygon::fillPolygon(const std::vector<std::array<double, 2>>& points_xy, int width, int height,
                                          double min_x, double min_y, std::vector<int8_t>& grid, int8_t value) const {
    if (points_xy.size() < 3) return;
    std::vector<std::pair<int, int>> grid_points;
    for (const auto& pt : points_xy) {
        int gx, gy;
        worldToGrid(pt[0], pt[1], min_x, min_y, gx, gy);
        grid_points.push_back({gx, gy});
    }
    const size_t n = grid_points.size();
    int min_y_grid = height, max_y_grid = 0;
    for (const auto& p : grid_points) {
        min_y_grid = std::min(min_y_grid, p.second);
        max_y_grid = std::max(max_y_grid, p.second);
    }
    for (int y = min_y_grid; y <= max_y_grid; ++y) {
        std::vector<int> intersections;
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            int y1 = grid_points[i].second;
            int y2 = grid_points[j].second;
            if ((y1 <= y && y < y2) || (y2 <= y && y < y1)) {
                int x1 = grid_points[i].first;
                int x2 = grid_points[j].first;
                int x = (y2 != y1) ? (x1 + (y - y1) * (x2 - x1) / (y2 - y1)) : x1;
                intersections.push_back(x);
            }
        }
        std::sort(intersections.begin(), intersections.end());
        for (size_t i = 0; i + 1 < intersections.size(); i += 2) {
            for (int x = intersections[i]; x <= intersections[i + 1]; ++x) {
                if (x >= 0 && x < width && y >= 0 && y < height)
                    grid[static_cast<size_t>(y) * width + x] = value;
            }
        }
    }
}

void GlobalPlannerfromPolygon::morphClose(std::vector<int8_t>& data, int width, int height, int radius, int iters) const {
    if (radius <= 0 || iters <= 0) return;
    auto dilate = [&](std::vector<int8_t>& src) {
        std::vector<int8_t> dst = src;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (src[static_cast<size_t>(y) * width + x] == 0) {
                    for (int j = -radius; j <= radius; ++j) {
                        for (int i = -radius; i <= radius; ++i) {
                            int nx = x + i, ny = y + j;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                                dst[static_cast<size_t>(ny) * width + nx] = 0;
                        }
                    }
                }
            }
        }
        src.swap(dst);
    };
    auto erode = [&](std::vector<int8_t>& src) {
        std::vector<int8_t> dst = src;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (src[static_cast<size_t>(y) * width + x] == 0) {
                    bool keep = true;
                    for (int j = -radius; j <= radius && keep; ++j) {
                        for (int i = -radius; i <= radius && keep; ++i) {
                            int nx = x + i, ny = y + j;
                            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                            if (src[static_cast<size_t>(ny) * width + nx] != 0) keep = false;
                        }
                    }
                    if (!keep) dst[static_cast<size_t>(y) * width + x] = static_cast<int8_t>(outside_value_);
                }
            }
        }
        src.swap(dst);
    };
    for (int k = 0; k < iters; ++k) { dilate(data); erode(data); }
}

void GlobalPlannerfromPolygon::generateOccupancyGrid() {
    if (polygon_vertices_.empty()) return;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& poly : polygon_vertices_) {
        for (const auto& pt : poly) {
            if (pt[0] < min_x) min_x = pt[0];
            if (pt[0] > max_x) max_x = pt[0];
            if (pt[1] < min_y) min_y = pt[1];
            if (pt[1] > max_y) max_y = pt[1];
        }
    }
    if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y)) return;
    int width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1);
    int height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1);
    std::vector<int8_t> grid(static_cast<size_t>(width) * height, static_cast<int8_t>(outside_value_));
    for (const auto& poly : polygon_vertices_) {
        if (poly.size() < 3) continue;
        fillPolygon(poly, width, height, min_x, min_y, grid, 0);
    }
    if (close_radius_ > 0 && close_iters_ > 0)
        morphClose(grid, width, height, close_radius_, close_iters_);
    occupancy_grid_.header.stamp = rclcpp::Clock().now();
    occupancy_grid_.header.frame_id = map_frame_;
    occupancy_grid_.info.map_load_time = occupancy_grid_.header.stamp;
    occupancy_grid_.info.resolution = static_cast<float>(resolution_);
    occupancy_grid_.info.width = static_cast<uint32_t>(width);
    occupancy_grid_.info.height = static_cast<uint32_t>(height);
    occupancy_grid_.info.origin.position.x = min_x;
    occupancy_grid_.info.origin.position.y = min_y;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;
    occupancy_grid_.data = std::move(grid);
    occupancy_grid_ready_ = true;
    std::cout << "[HDMap] Occupancy grid generated: " << width << "x" << height << ", resolution " << resolution_ << std::endl;
}

visualization_msgs::msg::MarkerArray GlobalPlannerfromPolygon::getPolygonLineStripMarkers(const std::string& frame_id) const {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = rclcpp::Time(0);
    clear_marker.ns = "polygon_map_line_strips";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear_marker);

    for (size_t i = 0; i < polygon_vertices_.size(); ++i) {
        const auto& poly = polygon_vertices_[i];
        if (poly.size() < 2) continue;
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = rclcpp::Time(0);
        line_marker.ns = "polygon_map_line_strips";
        line_marker.id = static_cast<int>(i);
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.15;
        line_marker.color.r = 0.2f;
        line_marker.color.g = 0.6f;
        line_marker.color.b = 0.9f;
        line_marker.color.a = 1.0f;
        line_marker.points.reserve(poly.size() + 1);
        for (const auto& pt : poly) {
            geometry_msgs::msg::Point p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = 0.0;
            line_marker.points.push_back(p);
        }
        if (!poly.empty()) {
            geometry_msgs::msg::Point close;
            close.x = poly[0][0];
            close.y = poly[0][1];
            close.z = 0.0;
            line_marker.points.push_back(close);
        }
        out.markers.push_back(line_marker);
    }
    return out;
}

GlobalPlannerfromPolygon::GlobalPlannerfromPolygon()
{
}

GlobalPlannerfromPolygon::~GlobalPlannerfromPolygon()
{
}