#include "../include/osmVisualizer/osmVisualizer.hpp"

OsmVisualizer::OsmVisualizer() : Node("OsmVisualizer")
{
  this->declare_parameter("map_path", "/home/atakan/Downloads/Town10.osm");
  this->declare_parameter("enable_inc_path_points", true);
  this->declare_parameter("interval", 2.0);
  
  // Occupancy grid parameters
  this->declare_parameter("resolution", 0.20);
  this->declare_parameter("close_radius", 0);
  this->declare_parameter("close_iters", 0);
  this->declare_parameter("outside_value", 100);
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("occupancy_output_topic", "occupancy_grid_complete_map");

  x_offset_ = 0.0;
  y_offset_ = 0.0;
  
  // Initialize occupancy grid parameters
  resolution_ = this->get_parameter("resolution").as_double();
  close_radius_ = this->get_parameter("close_radius").as_int();
  close_iters_ = this->get_parameter("close_iters").as_int();
  outside_value_ = this->get_parameter("outside_value").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  occupancy_output_topic_ = this->get_parameter("occupancy_output_topic").as_string();
  
  // Clamp outside value to valid range
  outside_value_ = std::max(-1, std::min(100, outside_value_));
  
  occupancy_grid_ready_ = false;
  occupancy_dirty_ = false;

  if (!readParameters())
    rclcpp::shutdown();

  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map", 10);
  array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/array", 10);
  occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid_complete_map_2", rclcpp::SystemDefaultsQoS());
  timer_ = this->create_wall_timer(200ms, std::bind(&OsmVisualizer::timer_callback, this));

  polygon_publisher_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>("/crosswalk_polygons", 10);
  road_elements_publisher_ = this->create_publisher<traffic_information_msgs::msg::RoadElementsCollection>("/road_elements", 10);

  lanelet::Origin origin({49, 8.4});
  lanelet::projection::LocalCartesianProjector projector(origin);
  lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);
  // LaneletMapPtr map = load(map_path_, projection::UtmProjector(Origin({49, 8.4})));

  // for (auto &point : map->pointLayer)
  // {
  //   point.x() = point.attribute("local_x").asDouble().value();
  //   point.y() = point.attribute("local_y").asDouble().value();
  // }

  for (auto &point : map->pointLayer)
  {
      point.x() = point.attribute("local_x").asDouble().value() + x_offset_;
      point.y() = point.attribute("local_y").asDouble().value() + y_offset_;
  }

  RCLCPP_INFO(this->get_logger(), "\033[1;32m----> OsmVisualizer_node initialized.\033[0m");
  fill_marker(map);
  fill_array_with_left_right(map);
  generateOccupancyGrid(map);
}

bool OsmVisualizer::readParameters()
{
  if (!this->get_parameter("map_path", map_path_))
  {
    std::cout << "Failed to read parameter 'map_path' " << std::endl;
    return false;
  }
  if (!this->get_parameter("enable_inc_path_points", enable_inc_path_points_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  if (!this->get_parameter("interval", interval_))
  {
    std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
    return false;
  }
  return true;
}

void OsmVisualizer::timer_callback()
{
  // publish until one subcribre that is the rviz2 for vizualisation and publish only once
  if (publisher_->get_subscription_count() > 0 && m_first)
  {
    publisher_->publish(m_marker_array);
    m_first = false;
  }
  // publish until one subcribre that is the ocupanccy grid code and publish only once
  if (array_publisher_->get_subscription_count() > 0 && m_second)
  {
    array_publisher_->publish(m_array);
    m_second = false;
  }
  
  // publish occupancy grid
  if (occupancy_grid_ready_)
  {
    publishOccupancyGrid();
  }

  if (!crosswalk_polygons.polygons.empty())
  {
    // publish until one subcribre that is the rviz2 for vizualisation and publish only once
    if (polygon_publisher_->get_subscription_count() > 0 && m_third)
    {
      polygon_publisher_->publish(crosswalk_polygons);
      m_third = false;
    }
    // publish until one subcribre
    if (road_elements_publisher_->get_subscription_count() > 0)
    {
      road_elements_publisher_->publish(road_elements);
    }
  }
}


void OsmVisualizer::writeToFile(const std_msgs::msg::Float64MultiArray &multi_array)
{
  std::ofstream file("data.txt");
  if (file.is_open())
  {
    for (size_t i = 0; i < multi_array.data.size(); ++i)
    {
      file << multi_array.data[i] << ",";
      if ((i + 1) % (multi_array.layout.dim[0].size) == 0)
        file << "\n";
      if ((i + 1) % (multi_array.layout.dim[1].size) == 0)
        file << "\n";
    }
    file.close();
  }
}

void OsmVisualizer::fill_array_with_left_right(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = t_map->laneletLayer.size();
  m_array.layout.dim[0].stride = t_map->laneletLayer.size() * 4;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 4;
  m_array.layout.dim[1].stride = 4;

  // Define an interpolation interval (distance between each interpolated point)
  double interval = 0.1; // Adjust this value based on your map resolution and needs

  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    // Check if the lanelet has the subtype 'crosswalk' and skip it
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() != lanelet::AttributeValueString::Crosswalk)
    {
      // Check if the lanelet has exactly four points on each boundary (rectangular structure)
      if (bounds[0].size() == 2 && bounds[1].size() == 2)
      {

        // std::cout << blue << "Rects  lanelet ID: " << ll.id() << reset << std::endl;

        size_t left_size = bounds[0].size();
        size_t right_size = bounds[1].size();

        // Interpolate between each pair of consecutive points
        for (size_t segment = 0; segment < left_size - 1; ++segment)
        {
          // Get start and end points of the current segment for left and right boundaries
          auto left_start = bounds[0][segment];
          auto left_end = bounds[0][segment + 1];
          auto right_start = bounds[1][segment];
          auto right_end = bounds[1][segment + 1];

          // Calculate the distance for the left and right segments
          double left_dist = std::sqrt(std::pow(left_end.x() - left_start.x(), 2) +
                                       std::pow(left_end.y() - left_start.y(), 2));
          double right_dist = std::sqrt(std::pow(right_end.x() - right_start.x(), 2) +
                                        std::pow(right_end.y() - right_start.y(), 2));

          // Determine the number of interpolation points based on the shortest segment
          int num_points = static_cast<int>(std::min(left_dist, right_dist) / interval);

          for (int i = 0; i <= num_points; ++i)
          {
            // Interpolate for the left boundary
            double t = static_cast<double>(i) / num_points;
            double left_x = left_start.x() + t * (left_end.x() - left_start.x());
            double left_y = left_start.y() + t * (left_end.y() - left_start.y());

            // Interpolate for the right boundary
            double right_x = right_start.x() + t * (right_end.x() - right_start.x());
            double right_y = right_start.y() + t * (right_end.y() - right_start.y());

            // Add the interpolated points to m_array
            m_array.data.push_back(left_x);
            m_array.data.push_back(left_y);
            m_array.data.push_back(right_x);
            m_array.data.push_back(right_y);
          }
        }
      }
      else
      {
        // If there are more than four points, just add the existing points without interpolation
        // std::cout << green << "Not rect lanelet ID: " << ll.id() << reset << std::endl;

        size_t size = std::min(bounds[0].size(), bounds[1].size());
        for (size_t i = 0; i < size; i++)
        {
          m_array.data.push_back(bounds[0][i].x());
          m_array.data.push_back(bounds[0][i].y());
          m_array.data.push_back(bounds[1][i].x());
          m_array.data.push_back(bounds[1][i].y());
        }
      }
    }
  }
}

double OsmVisualizer::getDistance(const lanelet::ConstLanelet &ll, size_t i)
{
  return std::sqrt(std::pow(ll.centerline2d()[i].x() - ll.centerline2d()[i + 1].x(), 2) + std::pow(ll.centerline2d()[i].y() - ll.centerline2d()[i + 1].y(), 2));
}

void OsmVisualizer::fill_marker(lanelet::LaneletMapPtr &t_map)
{

  size_t i = 0;
  int crosswalk_count = 0;             // Counter for crosswalk subtype
  int road_element_count = 0;          // Counter for road elements
  crosswalk_polygons.polygons.clear(); // Clear the crosswalk polygons
  road_elements.polygons.clear();      // Clear the road elements

  crosswalk_polygons.header.stamp = rclcpp::Clock{}.now();
  crosswalk_polygons.header.frame_id = "map";

  road_elements.header.stamp = rclcpp::Clock{}.now();
  road_elements.header.frame_id = "map";

  std_msgs::msg::ColorRGBA color;
  color.r = 0.6; // Red color
  color.g = 0.6; // No green
  color.b = 0.6; // No blue
  color.a = 0.5; // Full opacity

  crosswalk_polygons.colors.clear();
  crosswalk_polygons.colors.push_back(color);

  double base_polygon_z_offset = 0.0;   // Z-offset for the base crosswalk polygon
  double stripe_polygon_z_offset = 0.1; // Z-offset for the zebra crossing stripes

  // Iterate over the lanelets in the map
  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    // Check if the lanelet has the subtype 'crosswalk'
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
    {

      // cout the id of the crosswalk
      std::cout << "Crosswalk id: " << ll.id() << std::endl;
      polygon_msgs::msg::Polygon2D base_polygon;
      traffic_information_msgs::msg::RoadElements crosswalks_element;

      crosswalk_count++;   // Increment the crosswalk counter
      int num_stripes = 5; // Number of stripes in the zebra crossing

      double max_z = std::numeric_limits<double>::lowest();
      for (const auto &point : ll.leftBound())
      {
        if (point.z() > max_z)
        {
          max_z = point.z();
        }
      }

      // =================================================================================================
      // base polygon for road_elements_publisher
      // For the left bound
      for (const auto &point : ll.leftBound())
      {
        polygon_msgs::msg::Point2D p;
        p.x = point.x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = point.y();
        crosswalks_element.points.push_back(p); // Add to the polygon's points
      }

      // For the right bound
      const auto &right_bound = ll.rightBound();
      for (int i = right_bound.size() - 1; i >= 0; --i)
      {
        polygon_msgs::msg::Point2D p;
        p.x = right_bound[i].x(); // Convert from lanelet point to polygon_msgs Point2D
        p.y = right_bound[i].y();
        crosswalks_element.points.push_back(p); // Add to the polygon's points
      }

      crosswalks_element.points.push_back(crosswalks_element.points[0]);

      crosswalks_element.id = ll.id(); // Set the ID for the crosswal

      crosswalks_element.type = ll.attribute(lanelet::AttributeName::Subtype).value();

      road_elements.polygons.push_back(crosswalks_element);

      // std::cout << "crosswal type: " << ll.attribute(lanelet::AttributeName::Subtype).value() << std::endl;

      // =================================================================================================
      // stripe polygons

      // Calculate the total length of the left and right bounds
      double left_bound_length = 0.0;
      for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
      {
        left_bound_length += std::sqrt(
            std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
            std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2) +
            std::pow(ll.leftBound()[i + 1].z() - ll.leftBound()[i].z(), 2));
      }

      // std::cout << "----->left_bound_length: " << left_bound_length << std::endl;

      if (left_bound_length > 5.0)
      {
        num_stripes += static_cast<int>((left_bound_length - 5.0) / 1.0) * 1;
      }

      // std::cout << "----->num_stripes: " << num_stripes << std::endl;

      double stripe_length = left_bound_length / (2 * num_stripes);

      // Generate zebra stripes
      for (int stripe_idx = 0; stripe_idx < num_stripes; ++stripe_idx)
      {
        polygon_msgs::msg::Polygon2D stripe_polygon;
        stripe_polygon.z_offset = max_z;

        // Calculate the start and end points for the current stripe on the left and right bounds
        double start_dist = stripe_idx * 2 * stripe_length;
        double end_dist = start_dist + stripe_length;

        // Add points for the stripe from the left bound
        double accumulated_length = 0.0;
        polygon_msgs::msg::Point2D start_left, end_left;
        bool start_left_set = false, end_left_set = false;

        for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
        {
          double segment_length = std::sqrt(
              std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
              std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2));

          if (accumulated_length + segment_length > start_dist && !start_left_set)
          {
            double ratio = (start_dist - accumulated_length) / segment_length;
            start_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
            start_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
            start_left_set = true;
          }

          if (accumulated_length + segment_length > end_dist && !end_left_set)
          {
            double ratio = (end_dist - accumulated_length) / segment_length;
            end_left.x = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
            end_left.y = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
            end_left_set = true;
            break;
          }

          accumulated_length += segment_length;
        }

        if (start_left_set && end_left_set)
        {
          stripe_polygon.points.push_back(start_left);
          stripe_polygon.points.push_back(end_left);
        }

        // Add points for the stripe from the right bound
        accumulated_length = 0.0;
        polygon_msgs::msg::Point2D start_right, end_right;
        bool start_right_set = false, end_right_set = false;

        for (size_t i = 0; i < ll.rightBound().size() - 1; ++i)
        {
          double segment_length = std::sqrt(
              std::pow(ll.rightBound()[i + 1].x() - ll.rightBound()[i].x(), 2) +
              std::pow(ll.rightBound()[i + 1].y() - ll.rightBound()[i].y(), 2));

          if (accumulated_length + segment_length > start_dist && !start_right_set)
          {
            double ratio = (start_dist - accumulated_length) / segment_length;
            start_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
            start_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
            start_right_set = true;
          }

          if (accumulated_length + segment_length > end_dist && !end_right_set)
          {
            double ratio = (end_dist - accumulated_length) / segment_length;
            end_right.x = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
            end_right.y = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
            end_right_set = true;
            break;
          }

          accumulated_length += segment_length;
        }

        if (start_right_set && end_right_set)
        {
          stripe_polygon.points.push_back(end_right);
          stripe_polygon.points.push_back(start_right);
        }

        // Close the polygon by adding the first point again
        if (stripe_polygon.points.size() >= 4)
        {

          // Set color for the stripe
          std_msgs::msg::ColorRGBA stripe_color;
          stripe_color.r = 0.6;
          stripe_color.g = 0.6;
          stripe_color.b = 0.6;
          stripe_color.a = 0.8;

          crosswalk_polygons.colors.push_back(stripe_color);

          stripe_polygon.points.push_back(stripe_polygon.points[0]);
          crosswalk_polygons.polygons.push_back(stripe_polygon);
        }
      }
    }

    else
    {
      road_element_count++; // Increment the road element counter
      // For each bound (left and right), create a marker
      for (const auto &bound : bounds)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock{}.now();
        marker.ns = "lanelet";
        marker.id = i++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        // Add the points to the marker
        for (const auto &point : bound)
        {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          marker.points.push_back(p);
        }

        // Add the marker to the array
        m_marker_array.markers.push_back(marker);
      }
    }
  }
  std::cout << blue << "----> Number of crosswalk lanelets: " << crosswalk_count << reset << std::endl;
  std::cout << blue << "----> Number of road elements: " << road_element_count << reset << std::endl;
}

void OsmVisualizer::generateOccupancyGrid(lanelet::LaneletMapPtr &t_map)
{
  RCLCPP_INFO(this->get_logger(), "Generating occupancy grid from lanelets...");
  
  // 1) Compute bounding box from all lanelet points
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  auto updateBounds = [&](double x, double y) {
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
  };

  // Get bounds from all lanelets (excluding crosswalks)
  for (const auto &ll : t_map->laneletLayer)
  {
    // Skip crosswalks for occupancy grid
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
    {
      continue;
    }

    // Update bounds with left and right boundary points
    for (const auto &point : ll.leftBound())
    {
      updateBounds(point.x(), point.y());
    }
    for (const auto &point : ll.rightBound())
    {
      updateBounds(point.x(), point.y());
    }
  }

  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y))
  {
    RCLCPP_WARN(this->get_logger(), "Invalid bounds computed; skipping occupancy grid generation.");
    return;
  }

  // 2) Calculate grid dimensions
  int width = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
  int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;
  width = std::max(1, width);
  height = std::max(1, height);

  RCLCPP_INFO(this->get_logger(), "Grid dimensions: %dx%d, resolution: %.3f", width, height, resolution_);

  // 3) Initialize grid with outside value (occupied/unknown)
  std::vector<int8_t> grid(width * height, static_cast<int8_t>(outside_value_));

  // 4) Fill lanelet polygons with free space (0)
  for (const auto &ll : t_map->laneletLayer)
  {
    // Skip crosswalks for occupancy grid
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
    {
      continue;
    }

    // Create polygon from lanelet boundaries
    std::vector<lanelet::ConstPoint3d> polygon_points;
    
    // Add left boundary points
    for (const auto &point : ll.leftBound())
    {
      polygon_points.push_back(point);
    }
    
    // Add right boundary points in reverse order
    const auto &right_bound = ll.rightBound();
    for (int i = right_bound.size() - 1; i >= 0; --i)
    {
      polygon_points.push_back(right_bound[i]);
    }
    
    // Close the polygon
    if (!polygon_points.empty())
    {
      polygon_points.push_back(polygon_points[0]);
    }

    // Fill the polygon with free space
    fillLaneletPolygon(polygon_points, width, height, min_x, min_y, grid, 0);
  }

  // 5) Apply morphological closing to seal gaps
  if (close_radius_ > 0 && close_iters_ > 0)
  {
    morphClose(grid, width, height, close_radius_, close_iters_);
  }

  // 6) Fill occupancy grid message
  occupancy_grid_.header.stamp = now();
  occupancy_grid_.header.frame_id = frame_id_;
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
  occupancy_dirty_ = true;

  RCLCPP_INFO(this->get_logger(), "Occupancy grid generated successfully!");
}

void OsmVisualizer::publishOccupancyGrid()
{
  if (occupancy_grid_ready_)
  {
    occupancy_grid_publisher_->publish(occupancy_grid_);
    occupancy_dirty_ = false;
  }
}

void OsmVisualizer::worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const
{
  gx = static_cast<int>(std::floor((wx - min_x) / resolution_));
  gy = static_cast<int>(std::floor((wy - min_y) / resolution_));
}

void OsmVisualizer::drawLine(int x0, int y0, int x1, int y1, int width, int height,
                             std::vector<int8_t> &data, int8_t value) const
{
  auto inBounds = [&](int x, int y) { return x >= 0 && x < width && y >= 0 && y < height; };
  
  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  
  while (true)
  {
    if (inBounds(x, y)) data[y * width + x] = value;
    if (x == x1 && y == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
}

void OsmVisualizer::morphClose(std::vector<int8_t> &data, int width, int height, int radius, int iters) const
{
  if (radius <= 0 || iters <= 0) return;

  auto dilate = [&](std::vector<int8_t> &src) {
    std::vector<int8_t> dst = src;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (src[y * width + x] == 0) {
          for (int j = -radius; j <= radius; ++j) {
            for (int i = -radius; i <= radius; ++i) {
              int nx = x + i, ny = y + j;
              if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                dst[ny * width + nx] = 0;
            }
          }
        }
      }
    }
    src.swap(dst);
  };

  auto erode = [&](std::vector<int8_t> &src) {
    std::vector<int8_t> dst = src;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (src[y * width + x] == 0) {
          bool keep = true;
          for (int j = -radius; j <= radius && keep; ++j) {
            for (int i = -radius; i <= radius; ++i) {
              int nx = x + i, ny = y + j;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
              if (src[ny * width + nx] != 0) keep = false;
            }
          }
          if (!keep) dst[y * width + x] = static_cast<int8_t>(outside_value_);
        }
      }
    }
    src.swap(dst);
  };

  for (int k = 0; k < iters; ++k) { dilate(data); erode(data); }
}

void OsmVisualizer::fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points, int width, int height,
                                       double min_x, double min_y, std::vector<int8_t> &grid, int8_t value) const
{
  if (points.size() < 3) return;

  // Convert polygon points to grid coordinates
  std::vector<std::pair<int, int>> grid_points;
  for (const auto &point : points)
  {
    int gx, gy;
    worldToGrid(point.x(), point.y(), min_x, min_y, gx, gy);
    grid_points.push_back({gx, gy});
  }

  // Use scanline algorithm to fill polygon
  int min_y_grid = height, max_y_grid = 0;
  for (const auto &p : grid_points)
  {
    min_y_grid = std::min(min_y_grid, p.second);
    max_y_grid = std::max(max_y_grid, p.second);
  }

  for (int y = min_y_grid; y <= max_y_grid; ++y)
  {
    std::vector<int> intersections;
    
    // Find intersections with horizontal line
    for (size_t i = 0; i < grid_points.size() - 1; ++i)
    {
      int y1 = grid_points[i].second;
      int y2 = grid_points[i + 1].second;
      
      if ((y1 <= y && y < y2) || (y2 <= y && y < y1))
      {
        int x1 = grid_points[i].first;
        int x2 = grid_points[i + 1].first;
        int x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
        intersections.push_back(x);
      }
    }
    
    // Sort intersections and fill between pairs
    std::sort(intersections.begin(), intersections.end());
    for (size_t i = 0; i < intersections.size(); i += 2)
    {
      if (i + 1 < intersections.size())
      {
        for (int x = intersections[i]; x <= intersections[i + 1]; ++x)
        {
          if (x >= 0 && x < width && y >= 0 && y < height)
          {
            grid[y * width + x] = value;
          }
        }
      }
    }
  }
}
