#include <path_planning.hpp>

path_planning::path_planning() : Node("path_planning"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{
    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<int>("pathLength", 0);
    this->declare_parameter<double>("step_car", 0.0);
    this->declare_parameter<int>("tree_depth", 3);
    this->declare_parameter<int>("branching_factor", 5);
    this->declare_parameter<std::string>("map_path", "");
    this->declare_parameter<double>("x_offset", 0.0);
    this->declare_parameter<double>("y_offset", 0.0);
    this->declare_parameter<int>("start_lanelet_id", 0);
    this->declare_parameter<int>("end_lanelet_id", 0);

    // Occupancy grid parameters
    this->declare_parameter<double>("global_planner_resolution", 0.20);
    this->declare_parameter<int>("global_planner_close_radius", 0);
    this->declare_parameter<int>("global_planner_close_iters", 0);
    this->declare_parameter<int>("global_planner_outside_value", 100);
    this->declare_parameter<std::string>("global_planner_frame_id", "map");
    this->declare_parameter<std::string>("global_planner_occupancy_output_topic", "occupancy_grid_complete_map");

    this->get_parameter("maxSteerAngle", maxSteerAngle);
    this->get_parameter("wheelBase", wheelBase);
    this->get_parameter("axleToFront", axleToFront);
    this->get_parameter("axleToBack", axleToBack);
    this->get_parameter("width", width);
    this->get_parameter("pathLength", pathLength);
    this->get_parameter("step_car", step_car);
    this->get_parameter("tree_depth", tree_depth);
    this->get_parameter("branching_factor", branching_factor);
    this->get_parameter("map_path", map_path_);
    this->get_parameter("x_offset", x_offset_);
    this->get_parameter("y_offset", y_offset_);
    this->get_parameter("start_lanelet_id", start_lanelet_id_);
    this->get_parameter("end_lanelet_id", end_lanelet_id_);

    // Occupancy grid parameters
    this->get_parameter("global_planner_resolution", global_planner_resolution_);
    this->get_parameter("global_planner_close_radius", global_planner_close_radius_);
    this->get_parameter("global_planner_close_iters", global_planner_close_iters_);
    this->get_parameter("global_planner_outside_value", global_planner_outside_value_);
    this->get_parameter("global_planner_frame_id", global_planner_frame_id_);
    this->get_parameter("global_planner_occupancy_output_topic", global_planner_occupancy_output_topic_);

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&path_planning::obstacle_info_callback, this, std::placeholders::_1));

    // publisher for the occupancy grid of the obstacles
    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);
    
    real_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/none_real_traj", 10);

    real_trajectories_pub_2 = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_2", 10);

    all_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/all_available_paths", 10);

    sdv_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/sdv_trajectory", 10);

    global_planner_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/global_planner", 10);

    global_planner_occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        global_planner_occupancy_output_topic_, 10);

    // -------------> Initialize the shared pointers  <------------
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    car_state_ = std::make_shared<State>();
    grid_map_ = nullptr;
    current_node = nullptr;
    global_planner_ = std::make_shared<GlobalPlanner>(x_offset_, y_offset_, map_path_, start_lanelet_id_, end_lanelet_id_, global_planner_resolution_, global_planner_close_radius_, global_planner_close_iters_, global_planner_outside_value_, global_planner_frame_id_);

    // Create the vehicle geometry
    car_data_ = CarData(maxSteerAngle, wheelBase, axleToFront, axleToBack, width);
    car_data_.createVehicleGeometry();

    // log out parameters
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmaxSteerAngle: %f\033[0m", maxSteerAngle);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwheelBase: %f\033[0m", wheelBase);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToFront: %f\033[0m", axleToFront);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToBack: %f\033[0m", axleToBack);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwidth: %f\033[0m", width);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %d\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mtree_depth: %d\033[0m", tree_depth);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mbranching_factor: %d\033[0m", branching_factor);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmap_path: %s\033[0m", map_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mx_offset: %f\033[0m", x_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34my_offset: %f\033[0m", y_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstart_lanelet_id: %d\033[0m", start_lanelet_id_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mend_lanelet_id: %d\033[0m", end_lanelet_id_);

    // get the motion commans
    motionCommands();
    precomputeCommandSamples();
    all_waypoints_from_global_planner_ = global_planner_->getAllAllWaypointsStruct();
    publishGlobalPlanner();
    if (global_planner_->isOccupancyGridReady())
    {
        global_planner_occupancy_grid_ = global_planner_->getOccupancyGrid();
        publishGlobalPlannerOccupancyGrid();
        global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(global_planner_occupancy_grid_);
    }
}


path_planning::~path_planning()
{
}

// =============================
// get the state of the car
// =============================
void path_planning::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        car_state_->x = pose_tf.translation.x;
        car_state_->y = pose_tf.translation.y;
        car_state_->z = pose_tf.translation.z - 2.10;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_->heading = yaw;

        // Create/refresh the current node with updated car state
        vector<State> empty_trajectory = {*car_state_};
        current_node = std::make_shared<planner::Node>(*car_state_, empty_trajectory, 0.0, 0.0, 1, std::weak_ptr<planner::Node>());
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

// =============================
// publish the global planner
// =============================
void path_planning::publishGlobalPlanner()
{
    std::cout << green << "Publishing global planner" << reset << std::endl;
    std::cout << green << "Global planner size: " << all_waypoints_from_global_planner_.size() << reset << std::endl;
    global_planner_markers_.markers.clear();
    
    // Clear previous text markers
    visualization_msgs::msg::Marker clear_text;
    clear_text.header.frame_id = "map";
    clear_text.header.stamp = this->now();
    clear_text.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_text.ns = "global_planner_text";
    global_planner_markers_.markers.push_back(clear_text);
    
    for (size_t i = 0; i < all_waypoints_from_global_planner_.size(); i++)
    {
            // Create waypoint marker for main path from the global planner
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "global_planner";
            waypoint_marker.id = i;
            waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

            waypoint_marker.color.a = 0.8;
            
            // Color based on lane_sequence_id
            int seq_id = all_waypoints_from_global_planner_[i].lane_sequence_id;
            switch (seq_id) {
                case 0: // Main path - Blue
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 1: // First neighbor group - Green
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 2: // Second neighbor group - Red
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 3: // Third neighbor group - Yellow
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 4: // Fourth neighbor group - Magenta
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 5: // Fifth neighbor group - Cyan
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 6: // Sixth neighbor group - Orange
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.5;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 7: // Seventh neighbor group - Purple
                    waypoint_marker.color.r = 0.5;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 8: // Eighth neighbor group - Pink
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.75;
                    waypoint_marker.color.b = 0.8;
                    break;
                case 9: // Ninth neighbor group - Light Blue
                    waypoint_marker.color.r = 0.5;
                    waypoint_marker.color.g = 0.8;
                    waypoint_marker.color.b = 1.0;
                    break;
                default: // Higher sequence IDs - Cycle through colors
                    {
                        int color_index = seq_id % 10;
                        switch (color_index) {
                            case 0: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 1: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 0.0; break;
                            case 2: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 0.0; break;
                            case 3: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 0.0; break;
                            case 4: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 5: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 1.0; break;
                            case 6: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.5; waypoint_marker.color.b = 0.0; break;
                            case 7: waypoint_marker.color.r = 0.5; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 8: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.75; waypoint_marker.color.b = 0.8; break;
                            case 9: waypoint_marker.color.r = 0.5; waypoint_marker.color.g = 0.8; waypoint_marker.color.b = 1.0; break;
                        }
                    }
                    break;
            }

            waypoint_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            waypoint_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            waypoint_marker.pose.position.z = 0.0;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, all_waypoints_from_global_planner_[i].heading);
            waypoint_marker.pose.orientation.x = quaternion.x();
            waypoint_marker.pose.orientation.y = quaternion.y();
            waypoint_marker.pose.orientation.z = quaternion.z();
            waypoint_marker.pose.orientation.w = quaternion.w();

            waypoint_marker.scale.x = 0.6; // Arrow length
            waypoint_marker.scale.y = 0.2; // Arrow width
            waypoint_marker.scale.z = 0.2; // Arrow height

            global_planner_markers_.markers.push_back(waypoint_marker);
            
            // Create text marker for lane_sequence_id
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "global_planner_text";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text above the arrow
            text_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            text_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            text_marker.pose.position.z = 0.5; // Above the arrow
            
            // Set text content to lane_sequence_id
            text_marker.text = std::to_string(all_waypoints_from_global_planner_[i].lane_sequence_id);
            
            // Text styling
            text_marker.scale.z = 0.3; // Text size
            text_marker.color.a = 1.0;
            text_marker.color.r = 1.0; // White text
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            
            global_planner_markers_.markers.push_back(text_marker);
    }
    global_planner_publisher_->publish(global_planner_markers_);
}

// =============================
// publish the global planner occupancy grid
// =============================
void path_planning::publishGlobalPlannerOccupancyGrid()
{

    std::cout << green << "Publishing global planner occupancy grid" << reset << std::endl;
    std::cout << green << "Grid size: " << global_planner_occupancy_grid_.info.width << "x" 
                << global_planner_occupancy_grid_.info.height << ", resolution: " 
                << global_planner_occupancy_grid_.info.resolution << reset << std::endl;

    // 
    
    global_planner_occupancy_grid_publisher_->publish(global_planner_occupancy_grid_);

}

// =============================
// map combination & rescale for put obstacles in the global map
// =============================
void path_planning::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    if (!global_map_)
    {
        RCLCPP_ERROR(this->get_logger(), "Global map is not available");
        return;
    }
    if (msg->obstacles.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Obstacles are not available");
        return;
    }
    std::cout << green << "Obstacles are available and global map is available" << reset << std::endl;
    getCurrentRobotState();
    map_combination(msg);
}

cv::Mat path_planning::toMat(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    for (size_t i = 0; i < map.data.size(); i++)
    {
        if (map.data[i] == 0)
            im.data[i] = 254; // Free space
        else if (map.data[i] == 100)
            im.data[i] = 0; // Occupied space
        else
            im.data[i] = 205; // Unknown space
    }
    return im;
}

cv::Mat path_planning::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

void path_planning::map_combination(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    // print the obstacle collection size
    std::cout << green << "Obstacle collection size: " << msg->obstacles.size() << reset << std::endl;

    auto init_time = std::chrono::system_clock::now();
    // clean the rescaled_chunk_
    rescaled_chunk_->data.clear();

    // Compute grid_map_origin position
    State grid_map_origin;

    // Calculate the position based on the car's heading
    grid_map_origin.x = car_state_->x + forward_distance * cos(car_state_->heading);
    grid_map_origin.y = car_state_->y + forward_distance * sin(car_state_->heading);
    grid_map_origin.z = car_state_->z; // Same height as car's position
    grid_map_origin.heading = car_state_->heading;

    State white_square;

    // Calculate the position based on the car's heading
    white_square.x = car_state_->x + forward_distance_square * cos(car_state_->heading);
    white_square.y = car_state_->y + forward_distance_square * sin(car_state_->heading);
    white_square.z = car_state_->z; // Same height as car's position
    white_square.heading = car_state_->heading;

    // Convert car state to grid coordinates
    int car_x_grid = static_cast<int>((grid_map_origin.x - global_map_->info.origin.position.x) / global_map_->info.resolution);
    int car_y_grid = static_cast<int>((grid_map_origin.y - global_map_->info.origin.position.y) / global_map_->info.resolution);

    // Define chunk boundaries
    int min_x = std::max(0, car_x_grid - chunk_radius);
    int max_x = std::min(static_cast<int>(global_map_->info.width), car_x_grid + chunk_radius);
    int min_y = std::max(0, car_y_grid - chunk_radius);
    int max_y = std::min(static_cast<int>(global_map_->info.height), car_y_grid + chunk_radius);

    // Initialize the chunk grid
    nav_msgs::msg::OccupancyGrid chunk;
    chunk.header = global_map_->header;
    chunk.info.resolution = global_map_->info.resolution;
    chunk.info.width = max_x - min_x;
    chunk.info.height = max_y - min_y;
    chunk.info.origin.position.x = global_map_->info.origin.position.x + min_x * global_map_->info.resolution;
    chunk.info.origin.position.y = global_map_->info.origin.position.y + min_y * global_map_->info.resolution;
    chunk.info.origin.position.z = car_state_->z;
    chunk.info.origin.orientation.w = 1.0;

    chunk.data.resize(chunk.info.width * chunk.info.height, 0);

    for (int y = min_y; y < max_y; ++y)
    {
        for (int x = min_x; x < max_x; ++x)
        {
            int global_index = y * global_map_->info.width + x;
            int local_x = x - min_x;
            int local_y = y - min_y;
            int chunk_index = local_y * chunk.info.width + local_x;

            chunk.data[chunk_index] = global_map_->data[global_index];
        }
    }

    cv::Mat chunk_mat = toMat(chunk);
    cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);

    rescaled_chunk_->header = global_map_->header;
    rescaled_chunk_->info.resolution = 0.2;
    rescaled_chunk_->info.width = rescaled_chunk_mat.cols;
    rescaled_chunk_->info.height = rescaled_chunk_mat.rows;
    rescaled_chunk_->info.origin.position.x = chunk.info.origin.position.x;
    rescaled_chunk_->info.origin.position.y = chunk.info.origin.position.y;
    rescaled_chunk_->info.origin.position.z = car_state_->z ;
    rescaled_chunk_->info.origin.orientation.w = 1.0;

    rescaled_chunk_->data.resize(rescaled_chunk_->info.width * rescaled_chunk_->info.height, 0);

    for (int i = 0; i < rescaled_chunk_mat.rows * rescaled_chunk_mat.cols; i++)
    {
        if (rescaled_chunk_mat.data[i] == 254)
            rescaled_chunk_->data[i] = 0;
        else if (rescaled_chunk_mat.data[i] == 0)
            rescaled_chunk_->data[i] = 100;
        else
            rescaled_chunk_->data[i] = -1;
    }

    auto mark_grid = [&](int x, int y, int value)
    {
        if (x >= 0 && x < static_cast<int>(rescaled_chunk_->info.width) && y >= 0 && y < static_cast<int>(rescaled_chunk_->info.height))
        {
            rescaled_chunk_->data[y * rescaled_chunk_->info.width + x] = value; // Mark the cell
        }
    };

    auto inflate_point = [&](int x, int y, int radius, int value)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                if (dx * dx + dy * dy <= radius * radius)
                { // Circle equation
                    mark_grid(x + dx, y + dy, value);
                }
            }
        }
    };

    auto draw_inflated_line = [&](int x0, int y0, int x1, int y1, int radius, int value)
    {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            inflate_point(x0, y0, radius, value);

            if (error > 0)
            {
                x0 += x_inc;
                error -= dy;
            }
            else
            {
                y0 += y_inc;
                error += dx;
            }
        }
    };

    int inflation_radius = 2; // Inflated cells around the obstacles
    int value_to_mark = 100;

    // Transformation from lidar frame to map frame
    double cos_heading = cos(car_state_->heading);
    double sin_heading = sin(car_state_->heading);

    // Function to check if a point is inside a polygon using ray casting algorithm
    auto point_in_polygon = [&](int x, int y, const std::vector<std::pair<int, int>>& polygon) -> bool
    {
        bool inside = false;
        int j = polygon.size() - 1;
        
        for (int i = 0; i < static_cast<int>(polygon.size()); i++)
        {
            if (((polygon[i].second > y) != (polygon[j].second > y)) &&
                (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                 (polygon[j].second - polygon[i].second) + polygon[i].first))
            {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    };

    // Function to fill obstacle interiors with dark grid
    auto fill_obstacle_interior = [&](const std::vector<std::pair<int, int>>& polygon_vertices, int fill_value)
    {
        if (polygon_vertices.size() < 3) return; // Need at least 3 points for a polygon
        
        // Find bounding box of the polygon
        int min_x = polygon_vertices[0].first, max_x = polygon_vertices[0].first;
        int min_y = polygon_vertices[0].second, max_y = polygon_vertices[0].second;
        
        for (const auto& vertex : polygon_vertices)
        {
            min_x = std::min(min_x, vertex.first);
            max_x = std::max(max_x, vertex.first);
            min_y = std::min(min_y, vertex.second);
            max_y = std::max(max_y, vertex.second);
        }
        
        // Clamp to grid boundaries
        min_x = std::max(0, min_x);
        max_x = std::min(static_cast<int>(rescaled_chunk_->info.width) - 1, max_x);
        min_y = std::max(0, min_y);
        max_y = std::min(static_cast<int>(rescaled_chunk_->info.height) - 1, max_y);
        
        // Fill all points inside the polygon
        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                if (point_in_polygon(x, y, polygon_vertices))
                {
                    mark_grid(x, y, fill_value);
                }
            }
        }
    };

    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        const auto &obstacle = msg->obstacles[i];
        
        // Store polygon vertices in grid coordinates for filling
        std::vector<std::pair<int, int>> polygon_vertices;
        
        for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        {
            auto &current_point_lidar = obstacle.polygon.points[j];
            auto &next_point_lidar = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];

            geometry_msgs::msg::Point current_point_map;
            current_point_map.x = car_state_->x + cos_heading * current_point_lidar.x - sin_heading * current_point_lidar.y;
            current_point_map.y = car_state_->y + sin_heading * current_point_lidar.x + cos_heading * current_point_lidar.y;
            geometry_msgs::msg::Point next_point_map;
            next_point_map.x = car_state_->x + cos_heading * next_point_lidar.x - sin_heading * next_point_lidar.y;
            next_point_map.y = car_state_->y + sin_heading * next_point_lidar.x + cos_heading * next_point_lidar.y;

            int x0 = static_cast<int>((current_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y0 = static_cast<int>((current_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);
            int x1 = static_cast<int>((next_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y1 = static_cast<int>((next_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

            // Store vertex for polygon filling
            polygon_vertices.push_back({x0, y0});

            // Draw inflated boundary
            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
        
        // Fill the interior of the obstacle with dark grid (value 100)
        fill_obstacle_interior(polygon_vertices, value_to_mark);
    }

    // Calculate the car's position in the rescaled grid
    double cos_heading_white = cos(white_square.heading);
    double sin_heading_white = sin(white_square.heading);

    int car_x_rescaled = static_cast<int>((white_square.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
    int car_y_rescaled = static_cast<int>((white_square.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

    // Create a solid white square around the car using rasterization
    // This ensures complete coverage without gaps
    double half_size_meters = (square_size * rescaled_chunk_->info.resolution) / 2.0;
    
    // Define the four corners of the square in world coordinates
    std::vector<std::pair<double, double>> corners = {
        {-half_size_meters, -half_size_meters},
        {half_size_meters, -half_size_meters},
        {half_size_meters, half_size_meters},
        {-half_size_meters, half_size_meters}
    };
    
    // Rotate corners and convert to grid coordinates
    std::vector<std::pair<int, int>> grid_corners;
    for (const auto& corner : corners) {
        double rotated_x = corner.first * cos_heading_white - corner.second * sin_heading_white;
        double rotated_y = corner.first * sin_heading_white + corner.second * cos_heading_white;
        
        int grid_x = car_x_rescaled + static_cast<int>(std::round(rotated_x / rescaled_chunk_->info.resolution));
        int grid_y = car_y_rescaled + static_cast<int>(std::round(rotated_y / rescaled_chunk_->info.resolution));
        
        grid_corners.push_back({grid_x, grid_y});
    }
    
    // Find bounding box for white square
    int square_min_x = std::min({grid_corners[0].first, grid_corners[1].first, grid_corners[2].first, grid_corners[3].first});
    int square_max_x = std::max({grid_corners[0].first, grid_corners[1].first, grid_corners[2].first, grid_corners[3].first});
    int square_min_y = std::min({grid_corners[0].second, grid_corners[1].second, grid_corners[2].second, grid_corners[3].second});
    int square_max_y = std::max({grid_corners[0].second, grid_corners[1].second, grid_corners[2].second, grid_corners[3].second});
    
    // Fill the bounding box with free space
    for (int y = square_min_y; y <= square_max_y; ++y) {
        for (int x = square_min_x; x <= square_max_x; ++x) {
            if (x >= 0 && x < static_cast<int>(rescaled_chunk_->info.width) &&
                y >= 0 && y < static_cast<int>(rescaled_chunk_->info.height)) {
                rescaled_chunk_->data[y * rescaled_chunk_->info.width + x] = 0;
            }
        }
    }

    buildDistanceField();
    buildWaypointDistanceFields();
    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    grid_map_->setcarData(car_data_);

    // Publish the rescaled chunk
    if (occupancy_grid_pub_test_->get_subscription_count() > 0)
    {
        occupancy_grid_pub_test_->publish(*rescaled_chunk_);
    }

    // TreeFlat flat;
    // int best = generateTrajectoryTree_AStar_flat_map(current_node->Current_state, flat);
    // publishBestPathFromFlat(flat, best, 1); // green color for the flat implementation

    TreeFlat flat_map;
    int best_map = generateTrajectoryTree_AStar_flat_map_with_waypoints(current_node->Current_state, flat_map);
    // publishBestPathFromFlat(flat_map, best_map, 2); // blue color for the A* implementation with waypoints
    // publishAllPathsFromFlat(flat_map); // publish all available paths
    publishTrajectoryPath(flat_map, best_map); // publish chosen trajectory as nav_msgs::Path


    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time for path selection: " << duration << " ms" << reset << endl;
}

  
void path_planning::motionCommands()
{
    int direction = 1;
    motionCommand.clear(); // Clear any existing commands
    
    // Generate motion commands based on branching_factor
    if (branching_factor <= 0) return;
    
    if (branching_factor == 1)
    {
        // Only straight ahead
        motionCommand.push_back({0.0, static_cast<double>(direction)});
    }
    else
    {
        // Distribute steering angles evenly across the range
        double angle_step = (2.0 * car_data_.maxSteerAngle) / (branching_factor - 1);
        
        for (int i = 0; i < branching_factor; ++i)
        {
            double angle = -car_data_.maxSteerAngle + i * angle_step;
            motionCommand.push_back({angle, static_cast<double>(direction)});
            std::cout << green << "Steering angle: " << angle << " Direction: " << direction << reset << std::endl;
        }
    }
}

void path_planning::precomputeCommandSamples()
{
    precomputed_rel_.assign(motionCommand.size(), {});
    for (size_t ci = 0; ci < motionCommand.size(); ++ci) {
        const double steer = motionCommand[ci][0];
        const int    dir   = static_cast<int>(motionCommand[ci][1]);

        std::vector<RelSample> seq;
        seq.reserve((size_t)pathLength);

        // Build a canonical segment starting from the origin frame (0 pose)
        State s; s.x = 0.0; s.y = 0.0; s.z = 0.0; s.heading = 0.0;
        for (int k = 0; k < pathLength; ++k) {
            s = car_data_.getVehicleStep(s, steer, dir, step_car);
            seq.push_back(RelSample{ s.x, s.y, s.heading });
        }
        precomputed_rel_[ci] = std::move(seq);
    }
}

// =============================
// generate the trajectory based on the A* algorithm
// =============================
int path_planning::generateTrajectoryTree_AStar_flat_map(const State& root_state, TreeFlat& out)
{
    out.nodes.clear();
    out.leaves.clear();

    const int B = std::max(1, branching_factor);
    const int D = std::max(0, tree_depth);
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    // Ensure motion samples exist for current commands
    if ((int)precomputed_rel_.size() != B) {
        precomputeCommandSamples();
    }

    // Precompute start-frame axes (for forward/lateral projections)
    const double cs0 = std::cos(root_state.heading);
    const double ss0 = std::sin(root_state.heading);

    // Reserve generously, but finite
    size_t max_nodes = 1, powB = 1;
    for (int d = 0; d < EFFECTIVE_DEPTH; ++d) { powB *= (size_t)B; max_nodes += powB; }
    max_nodes = std::min(max_nodes, (size_t)500000);
    out.nodes.reserve(max_nodes);

    // Root node
    FlatNode root;
    root.state  = root_state;
    root.parent = -1;
    root.steer  = 0.0;
    root.dir    = 1;
    root.depth  = 0;
    root.cost   = 0.0;     // g(root)
    out.nodes.push_back(root);

    // Best goal found so far
    int    best_goal_idx   = -1;
    double best_goal_cost  = std::numeric_limits<double>::infinity();

    // OPEN and best-g (duplicate suppression on lattice)
    std::priority_queue<PQItem> open;
    std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g;

    auto stateKey = [&](const State& s)->LatticeKey {
        return LatticeKey{ s.gridx, s.gridy, heading_bin(s.heading) };
    };

    // Heuristic lower bound from depth d to EFFECTIVE_DEPTH (reward only)
    auto h_lower_bound = [&](int depth)->double {
        const int remaining_segments = EFFECTIVE_DEPTH - depth;
        if (remaining_segments <= 0) return 0.0;
        const int remaining_steps = remaining_segments * pathLength;
        // Best case: straight forward progress each step
        return -W_FORWARD * (remaining_steps * step_car);
    };

    // Push root
    {
        LatticeKey k = stateKey(root.state);
        best_g[k] = 0.0;
        const double f0 = 0.0 + h_lower_bound(0);
        open.push(PQItem{0, f0, 0.0});
    }

    // Expand parent->child for motion index ci (returns child idx or -1)
    auto expand_one = [&](int parent_idx, size_t ci)->int {
        const FlatNode& parent = out.nodes[parent_idx];

        // rotate once for parent.heading
        const double cp = std::cos(parent.state.heading);
        const double sp = std::sin(parent.state.heading);

        const auto& seq = precomputed_rel_[ci];

        State   last = parent.state;
        double  obs_pen_sum = 0.0;  // clearance penalty accumulator this segment

        for (int k = 0; k < pathLength; ++k) {
            const auto& r = seq[k];

            State ns;
            ns.x = parent.state.x + cp * r.x - sp * r.y;
            ns.y = parent.state.y + sp * r.x + cp * r.y;
            ns.z = parent.state.z;
            ns.heading = parent.state.heading + r.heading;

            auto cell = grid_map_->toCellID(ns);
            ns.gridx = std::get<0>(cell);
            ns.gridy = std::get<1>(cell);

            // hard collision check (your predicate)
            if (grid_map_->isSingleStateCollisionFreeImproved(ns)) {
                return -1; // reject whole segment
            }

            // soft clearance penalty using distance field
            const double d = clearanceMeters(ns.gridx, ns.gridy); // meters
            if (d < SAFE_CLEAR) obs_pen_sum += (SAFE_CLEAR - d);

            last = ns;
        }

        // Child node
        FlatNode child;
        child.state  = last;
        child.parent = parent_idx;
        child.depth  = parent.depth + 1;
        child.steer  = motionCommand[ci][0];
        child.dir    = (int)motionCommand[ci][1];

        // Costs
        const double steer_pen  = W_STEER   * std::fabs(child.steer);
        const double dsteer_pen = W_DSTEER  * std::fabs(child.steer - parent.steer);

        // forward reward in start-frame
        const double dx = (last.x - parent.state.x);
        const double dy = (last.y - parent.state.y);
        const double forward_inc =  dx * cs0 + dy * ss0;

        // average clearance penalty across steps
        const double obs_pen = W_OBS * (obs_pen_sum / std::max(1, pathLength));

        const double g_child = out.nodes[parent_idx].cost
                             + steer_pen
                             + dsteer_pen
                             + obs_pen
                             - W_FORWARD * forward_inc;

        child.cost = g_child; // store g

        // Duplicate suppression on lattice key
        LatticeKey ck = stateKey(child.state);
        auto it = best_g.find(ck);
        if (it != best_g.end() && g_child >= it->second - 1e-12) {
            return -1; // dominated
        }
        best_g[ck] = g_child;

        out.nodes.push_back(child);
        return static_cast<int>(out.nodes.size()) - 1;
    };

    // A* loop
    while (!open.empty())
    {
        PQItem cur = open.top(); open.pop();

        const int idx   = cur.idx;
        const auto& fn  = out.nodes[idx];
        const double g  = fn.cost;
        const int    d  = fn.depth;

        // stale entry?
        if (std::fabs(g - cur.g_copy) > 1e-12) continue;

        // Goal at EFFECTIVE_DEPTH → add terminal terms and maybe terminate
        if (d == EFFECTIVE_DEPTH)
        {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err = std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total = g
                               + W_LAT  * std::fabs(lateral)
                               + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx  = idx;
            }

            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) break;
            continue;
        }

        // Expand children
        bool produced_child = false;
        for (size_t ci = 0; ci < motionCommand.size(); ++ci)
        {
            const int child_idx = expand_one(idx, ci);
            if (child_idx < 0) continue;
            produced_child = true;

            const auto& ch = out.nodes[child_idx];
            const double h = h_lower_bound(ch.depth);
            open.push(PQItem{child_idx, ch.cost + h, ch.cost});
        }

        // Dead-end → treat as candidate goal too
        if (!produced_child)
        {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err = std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total = g
                               + W_LAT  * std::fabs(lateral)
                               + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx  = idx;
            }
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) break;
        }
    }

    // Keep the best leaf for downstream publishing
    out.leaves.clear();
    if (best_goal_idx >= 0) out.leaves.push_back(best_goal_idx);

    // BFS output for the number of nodes at each depth
    std::vector<int> per_depth(EFFECTIVE_DEPTH + 1, 0);
    for (const auto& n : out.nodes)
        if (n.depth > 0 && n.depth <= EFFECTIVE_DEPTH) per_depth[n.depth]++;

    return best_goal_idx;
}

int path_planning::generateTrajectoryTree_AStar_flat_map_with_waypoints(const State& root_state, TreeFlat& out)
{
    out.nodes.clear();
    out.leaves.clear();

    const int B = std::max(1, branching_factor);
    const int D = std::max(0, tree_depth);
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    // Ensure motion samples exist for current commands
    if ((int)precomputed_rel_.size() != B) {
        precomputeCommandSamples();
    }

    // Start-frame axes (for forward/lateral projections at termination)
    const double cs0 = std::cos(root_state.heading);
    const double ss0 = std::sin(root_state.heading);

    // Reserve roughly B^(depth) nodes (capped)
    size_t max_nodes = 1, powB = 1;
    for (int d = 0; d < EFFECTIVE_DEPTH; ++d) { powB *= (size_t)B; max_nodes += powB; }
    max_nodes = std::min(max_nodes, (size_t)500000);
    out.nodes.reserve(max_nodes);

    // Root
    FlatNode root;
    root.state  = root_state;
    root.parent = -1;
    root.steer  = 0.0;
    root.dir    = 1;
    root.depth  = 0;
    root.cost   = 0.0;     // g(root)
    out.nodes.push_back(root);

    // Best goal found so far
    int    best_goal_idx   = -1;
    double best_goal_cost  = std::numeric_limits<double>::infinity();

    // OPEN and best-g (duplicate suppression on lattice)
    std::priority_queue<PQItem> open;
    std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g;

    auto stateKey = [&](const State& s)->LatticeKey {
        return LatticeKey{ s.gridx, s.gridy, heading_bin(s.heading) };
    };

    // Admissible LB heuristic: only forward reward remaining
    auto h_lower_bound = [&](int depth)->double {
        const int remaining_segments = EFFECTIVE_DEPTH - depth;
        if (remaining_segments <= 0) return 0.0;
        const int remaining_steps = remaining_segments * pathLength;
        return -W_FORWARD * (remaining_steps * step_car);
    };

    // Push root
    {
        LatticeKey k = stateKey(root.state);
        best_g[k] = 0.0;
        const double f0 = 0.0 + h_lower_bound(0);
        open.push(PQItem{0, f0, 0.0});
    }

    // Safe sampler for waypoint distance fields (returns meters; 0 if out of range/none)
    auto sample_wp_dist = [&](int gx, int gy)->double {
        // Prefer prio-1 if available in this chunk, else prio-2, else no attraction
        if (has_wp1_ && !dist_wp1_m_.empty()) {
            if (gy >= 0 && gy < dist_wp1_m_.rows && gx >= 0 && gx < dist_wp1_m_.cols) {
                return (double)dist_wp1_m_.at<float>(gy, gx) * W_WP1;
            }
            return 0.0;
        } else if (has_wp2_ && !dist_wp2_m_.empty()) {
            if (gy >= 0 && gy < dist_wp2_m_.rows && gx >= 0 && gx < dist_wp2_m_.cols) {
                return (double)dist_wp2_m_.at<float>(gy, gx) * W_WP2;
            }
            return 0.0;
        }
        return 0.0;
    };

    // Expand parent->child for motion index ci (returns child idx or -1)
    auto expand_one = [&](int parent_idx, size_t ci)->int {
        const FlatNode& parent = out.nodes[parent_idx];

        // rotate once for parent.heading
        const double cp = std::cos(parent.state.heading);
        const double sp = std::sin(parent.state.heading);

        const auto& seq = precomputed_rel_[ci];

        State   last = parent.state;
        double  obs_pen_sum = 0.0;  // clearance penalty accumulator this segment
        double  wp_pen_sum  = 0.0;  // waypoint attraction accumulator (meters * weight)

        for (int k = 0; k < pathLength; ++k) {
            const auto& r = seq[k];

            State ns;
            ns.x = parent.state.x + cp * r.x - sp * r.y;
            ns.y = parent.state.y + sp * r.x + cp * r.y;
            ns.z = parent.state.z;
            ns.heading = parent.state.heading + r.heading;

            auto cell = grid_map_->toCellID(ns);
            ns.gridx = std::get<0>(cell);
            ns.gridy = std::get<1>(cell);

            // Hard collision check.
            // NOTE: if isSingleStateCollisionFreeImproved() returns "true means collision-free",
            //       invert the condition below (i.e., if (!collisionFree) reject).
            if (grid_map_->isSingleStateCollisionFreeImproved(ns)) {
                return -1; // reject whole segment on collision (adjust if API semantics differ)
            }

            // Soft clearance penalty using distance field
            const double d = clearanceMeters(ns.gridx, ns.gridy); // meters
            if (d < SAFE_CLEAR) obs_pen_sum += (SAFE_CLEAR - d);
            
            // Early termination if clearance is too low to avoid straight-line paths in curves
            if (d < SAFE_CLEAR * 0.6) {  // Quit if clearance is less than 60% of safe clearance (0.48m)
                return -1; // reject whole segment on insufficient clearance
            }

            // Waypoint attraction (distance to prio-1 if present, else prio-2)
            wp_pen_sum += sample_wp_dist(ns.gridx, ns.gridy);

            last = ns;
        }

        // Child node
        FlatNode child;
        child.state  = last;
        child.parent = parent_idx;
        child.depth  = parent.depth + 1;
        child.steer  = motionCommand[ci][0];
        child.dir    = (int)motionCommand[ci][1];

        // Costs
        const double steer_pen  = W_STEER   * std::fabs(child.steer);
        const double dsteer_pen = W_DSTEER  * std::fabs(child.steer - parent.steer);
        
        // Extra penalty for straight-ahead motion when waypoints are available AND obstacles are nearby
        double straight_penalty = 0.0;
        if (std::abs(child.steer) < 0.01 && (has_wp1_ || has_wp2_)) {
            // Only penalize straight motion if there are obstacles nearby (low clearance)
            double avg_clearance = obs_pen_sum / std::max(1, pathLength);
            if (avg_clearance > 0.1) { // If there's significant obstacle penalty, add straight penalty
                straight_penalty = 2.0; // Reduced penalty, only when obstacles present
            }
        }

        // forward reward measured in the start frame
        const double dx = (last.x - parent.state.x);
        const double dy = (last.y - parent.state.y);
        const double forward_inc =  dx * cs0 + dy * ss0;

        // Average penalties over steps for scale stability
        const double obs_pen = W_OBS * (obs_pen_sum / std::max(1, pathLength));
        const double wp_pen  = (wp_pen_sum / std::max(1, pathLength)); // already includes W_WP{1,2}

        const double g_child = out.nodes[parent_idx].cost
                             + steer_pen
                             + dsteer_pen
                             + obs_pen
                             + wp_pen
                             + straight_penalty
                             - W_FORWARD * forward_inc;

        child.cost = g_child; // store g

        // Duplicate suppression on lattice key
        LatticeKey ck = stateKey(child.state);
        auto it = best_g.find(ck);
        if (it != best_g.end() && g_child >= it->second - 1e-12) {
            return -1; // dominated
        }
        best_g[ck] = g_child;

        out.nodes.push_back(child);
        return static_cast<int>(out.nodes.size()) - 1;
    };

    // A* loop
    while (!open.empty())
    {
        PQItem cur = open.top(); open.pop();

        const int idx   = cur.idx;
        const auto& fn  = out.nodes[idx];
        const double g  = fn.cost;
        const int    d  = fn.depth;

        // stale entry?
        if (std::fabs(g - cur.g_copy) > 1e-12) continue;

        // Goal at EFFECTIVE_DEPTH → add terminal terms and maybe terminate
        if (d == EFFECTIVE_DEPTH)
        {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err = std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total = g
                               + W_LAT  * std::fabs(lateral)
                               + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx  = idx;
            }

            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) break;
            continue;
        }

        // Expand children
        bool produced_child = false;
        for (size_t ci = 0; ci < motionCommand.size(); ++ci)
        {
            const int child_idx = expand_one(idx, ci);
            if (child_idx < 0) continue;
            produced_child = true;

            const auto& ch = out.nodes[child_idx];
            const double h = h_lower_bound(ch.depth);
            open.push(PQItem{child_idx, ch.cost + h, ch.cost});
        }

        // Dead-end → treat as candidate goal too
        if (!produced_child)
        {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err = std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total = g
                               + W_LAT  * std::fabs(lateral)
                               + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx  = idx;
            }
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) break;
        }
    }

    // Collect all leaf nodes (nodes at maximum depth) for visualization
    out.leaves.clear();
    for (size_t i = 0; i < out.nodes.size(); ++i) {
        if (out.nodes[i].depth == EFFECTIVE_DEPTH) {
            out.leaves.push_back(static_cast<int>(i));
        }
    }
    
    // Also keep track of the best leaf
    if (best_goal_idx >= 0) {
        // Ensure best goal is in leaves (it should be if it reached max depth)
        bool found = false;
        for (int leaf : out.leaves) {
            if (leaf == best_goal_idx) {
                found = true;
                break;
            }
        }
        if (!found) {
            out.leaves.push_back(best_goal_idx);
        }
    }

    return best_goal_idx;
}


// =============================
//  helper functions for the A* algorithm
// =============================
inline void path_planning::build_chain_indices(
    const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const
{
    chain.clear();
    for (int i = leaf_idx; i != -1; i = flat.nodes[i].parent) chain.push_back(i);
    std::reverse(chain.begin(), chain.end()); // root -> leaf
}

void path_planning::buildDistanceField()
{
    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) {
        dist_m_.release();
        return;
    }

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution; // 0.2 in your setup

    // Build binary image for distance transform: free=255, else=0 (occupied OR unknown)
    cv::Mat bin(H, W, CV_8UC1);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int8_t v = rescaled_chunk_->data[y * W + x];
            // Your convention: 0=free, 100=occupied, -1=unknown
            bin.at<uint8_t>(y, x) = (v == 0) ? 255 : 0;
        }
    }

    // Distance transform in pixels
    cv::Mat dist_px;
    cv::distanceTransform(bin, dist_px, cv::DIST_L2, 3);

    // Convert pixels to meters
    dist_m_.create(H, W, CV_32FC1);
    const float scale = static_cast<float>(res);
    for (int y = 0; y < H; ++y) {
        const float* src = dist_px.ptr<float>(y);
        float*       dst = dist_m_.ptr<float>(y);
        for (int x = 0; x < W; ++x) dst[x] = src[x] * scale;
    }
}

inline double path_planning::clearanceMeters(int gx, int gy) const
{
    if (dist_m_.empty()) return 0.0;
    if (gy < 0 || gy >= dist_m_.rows || gx < 0 || gx >= dist_m_.cols) return 0.0;
    return static_cast<double>(dist_m_.at<float>(gy, gx));
}

void path_planning::buildWaypointDistanceFields()
{
    dist_wp1_m_.release();
    dist_wp2_m_.release();
    has_wp1_ = false;
    has_wp2_ = false;

    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) return;

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution;

    // Binary canvases for each priority: 255 where path pixels live, 0 elsewhere
    cv::Mat bin1(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat bin2(H, W, CV_8UC1, cv::Scalar(0));

    auto worldToGrid = [&](double x, double y, int& gx, int& gy) {
        gx = static_cast<int>((x - rescaled_chunk_->info.origin.position.x) / res);
        gy = static_cast<int>((y - rescaled_chunk_->info.origin.position.y) / res);
    };

    // Helper to draw a thick line in grid space
    auto drawThickLine = [&](cv::Mat& img, int x0, int y0, int x1, int y1, int radius) {
        cv::LineIterator it(img, cv::Point(x0, y0), cv::Point(x1, y1));
        for (int i = 0; i < it.count; ++i, ++it) {
            cv::circle(img, it.pos(), radius, cv::Scalar(255), cv::FILLED);
        }
    };

    // Group consecutive waypoints by priority and draw lines between neighbors
    auto drawByPriority = [&](int prio, cv::Mat& canvas, bool& has_any) {
        std::vector<cv::Point> pts;
        pts.reserve(all_waypoints_from_global_planner_.size());

        // Collect all in-chunk points for this priority
        for (const auto& p : all_waypoints_from_global_planner_) {
            if (p.priority != prio) continue;
            int gx, gy;
            worldToGrid(p.x, p.y, gx, gy);
            if (gx >= 0 && gx < W && gy >= 0 && gy < H) {
                pts.emplace_back(gx, gy);
            }
        }

        // Draw segments between consecutive in-chunk points
        if (pts.size() >= 1) {
            has_any = true;
            const int rad = std::max(1, (int)std::round(WP_STROKE_RADIUS_CELLS));
            // Densify by connecting consecutive points
            for (size_t i = 1; i < pts.size(); ++i) {
                drawThickLine(canvas, pts[i-1].x, pts[i-1].y, pts[i].x, pts[i].y, rad);
            }
            // Also mark isolated singletons
            for (const auto& q : pts) {
                cv::circle(canvas, q, rad, cv::Scalar(255), cv::FILLED);
            }
        }
    };

    drawByPriority(1, bin1, has_wp1_);
    drawByPriority(2, bin2, has_wp2_);

    // If there is no prio-1 in chunk, keep bin1 empty; same for prio-2.
    auto toMetersDist = [&](const cv::Mat& bin, cv::Mat& out_m) {
        if (cv::countNonZero(bin) == 0) {
            out_m.release(); // no geometry to attract to
            return;
        }
        cv::Mat inv; // distanceTransform wants non-zero free area as source; we want distance TO the path
        // Build "feature" mask: 255 at path pixels, 0 elsewhere; we want distance TO those features,
        // so invert to a mask where features are 0 and background is 255, then DT that.
        cv::Mat feat = 255 - bin;

        cv::Mat dist_px;
        cv::distanceTransform(feat, dist_px, cv::DIST_L2, 3);

        out_m.create(bin.rows, bin.cols, CV_32FC1);
        const float scale = static_cast<float>(res);
        for (int y = 0; y < bin.rows; ++y) {
            const float* src = dist_px.ptr<float>(y);
            float*       dst = out_m.ptr<float>(y);
            for (int x = 0; x < bin.cols; ++x) dst[x] = src[x] * scale;
        }
    };

    toMetersDist(bin1, dist_wp1_m_);
    toMetersDist(bin2, dist_wp2_m_);

    // Reconcile availability flags with actual outputs
    has_wp1_ = has_wp1_ && !dist_wp1_m_.empty();
    has_wp2_ = has_wp2_ && !dist_wp2_m_.empty();
}


// =============================
// publish the trajectory
// =============================
void path_planning::publishBestPathFromFlat(const TreeFlat& flat, int leaf_idx, int color_idx)
{
    if (color_idx == 1) 
    {
        if (leaf_idx < 0 || real_trajectories_pub_->get_subscription_count() == 0) return;
    }
    else if (color_idx == 2)
    {
        if (leaf_idx < 0 || real_trajectories_pub_2->get_subscription_count() == 0) return;
    }


    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    visualization_msgs::msg::MarkerArray msg;

    // clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "real_trajectories"; msg.markers.push_back(clear);
    clear.ns = "real_endpoints";     msg.markers.push_back(clear);
    clear.ns = "real_trajectory_labels"; msg.markers.push_back(clear);

    // helper to map (steer,dir) -> precomputed index
    auto find_cmd_index = [&](double steer, int dir)->int{
        for (size_t i = 0; i < motionCommand.size(); ++i)
            if (dir == (int)motionCommand[i][1] && std::abs(steer - motionCommand[i][0]) < 1e-9)
                return (int)i;
        return -1;
    };

    // line marker
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = this->now();
    line.ns = "real_trajectories";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;

    // chose btw to color to know wich implementation is used
    if (color_idx == 0) {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    } else if (color_idx == 1) {
        line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0;
    } else if (color_idx == 2) {
        line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0;
    }
    else {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    }

    line.color.a = 1.0;

    // start at the true root pose so the polyline includes the origin point
    State seg_start = flat.nodes[ chain.front() ].state;
    {
        geometry_msgs::msg::Point p;
        p.x = seg_start.x; p.y = seg_start.y; p.z = seg_start.z;
        line.points.push_back(p);
    }

    // march along all segments root->leaf and append every step
    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[ chain[k] ];
        const int ci = find_cmd_index(fn.steer, fn.dir);

        if (ci >= 0 && ci < (int)precomputed_rel_.size() &&
            (int)precomputed_rel_[ci].size() == pathLength)
        {
            const double c0 = std::cos(seg_start.heading);
            const double s0 = std::sin(seg_start.heading);

            for (int i = 0; i < pathLength; ++i) {
                const auto& r = precomputed_rel_[ci][i];
                geometry_msgs::msg::Point p;
                p.x = seg_start.x + c0 * r.x - s0 * r.y;
                p.y = seg_start.y + s0 * r.x + c0 * r.y;
                p.z = seg_start.z;
                line.points.push_back(p);
            }
            // advance start to the end of this segment
            const auto& rlast = precomputed_rel_[ci].back();
            seg_start.x += c0 * rlast.x - s0 * rlast.y;
            seg_start.y += s0 * rlast.x + c0 * rlast.y;
            seg_start.heading += rlast.heading;
        }
        else
        {
            // fallback: re-simulate this one segment
            State s = seg_start;
            for (int i = 0; i < pathLength; ++i) {
                s = car_data_.getVehicleStep(s, fn.steer, fn.dir, step_car);
                geometry_msgs::msg::Point p;
                p.x = s.x; p.y = s.y; p.z = s.z;
                line.points.push_back(p);
            }
            seg_start = s;
        }
    }

    msg.markers.push_back(line);

    // endpoint sphere
    visualization_msgs::msg::Marker endpoint;
    endpoint.header.frame_id = "map";
    endpoint.header.stamp = this->now();
    endpoint.ns = "real_endpoints";
    endpoint.id = 1;
    endpoint.type = visualization_msgs::msg::Marker::SPHERE;
    endpoint.action = visualization_msgs::msg::Marker::ADD;
    endpoint.pose.position.x = seg_start.x;
    endpoint.pose.position.y = seg_start.y;
    endpoint.pose.position.z = seg_start.z + 0.2;
    endpoint.scale.x = 0.2; endpoint.scale.y = 0.2; endpoint.scale.z = 0.2;
    endpoint.color = line.color; endpoint.color.a = 0.8;
    msg.markers.push_back(endpoint);

    if (color_idx == 1) 
    {
        real_trajectories_pub_->publish(msg);
    }
    else if (color_idx == 2)
    {
        real_trajectories_pub_2->publish(msg);
    }
}


void path_planning::publishAllPathsFromFlat(const TreeFlat& flat)
{
    if (all_paths_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray msg;

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "all_paths";
    msg.markers.push_back(clear);

    // Helper to map (steer,dir) -> precomputed index
    auto find_cmd_index = [&](double steer, int dir)->int{
        for (size_t i = 0; i < motionCommand.size(); ++i)
            if (dir == (int)motionCommand[i][1] && std::abs(steer - motionCommand[i][0]) < 1e-9)
                return (int)i;
        return -1;
    };

    // Publish all paths from root to each leaf
    for (size_t leaf_idx = 0; leaf_idx < flat.leaves.size(); ++leaf_idx)
    {
        int leaf = flat.leaves[leaf_idx];
        if (leaf < 0) continue;

        // Build chain root->leaf
        std::vector<int> chain;
        build_chain_indices(flat, leaf, chain);

        // Create line marker for this path
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "all_paths";
        line.id = static_cast<int>(leaf_idx);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05; // Thinner lines for all paths

        // Color based on path index (cycle through colors)
        int color_idx = leaf_idx % 6;
        switch (color_idx) {
            case 0: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; break; // Red
            case 1: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; break; // Green
            case 2: line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0; break; // Blue
            case 3: line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; break; // Yellow
            case 4: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0; break; // Magenta
            case 5: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; break; // Cyan
        }
        line.color.a = 0.7; // Semi-transparent

        // Start at the root pose
        State seg_start = flat.nodes[chain.front()].state;
        {
            geometry_msgs::msg::Point p;
            p.x = seg_start.x; p.y = seg_start.y; p.z = seg_start.z;
            line.points.push_back(p);
        }

        // March along all segments root->leaf
        for (size_t k = 1; k < chain.size(); ++k)
        {
            const auto& fn = flat.nodes[chain[k]];
            const int ci = find_cmd_index(fn.steer, fn.dir);

            if (ci >= 0 && ci < (int)precomputed_rel_.size() &&
                (int)precomputed_rel_[ci].size() == pathLength)
            {
                const double c0 = std::cos(seg_start.heading);
                const double s0 = std::sin(seg_start.heading);

                for (int i = 0; i < pathLength; ++i) {
                    const auto& r = precomputed_rel_[ci][i];
                    geometry_msgs::msg::Point p;
                    p.x = seg_start.x + c0 * r.x - s0 * r.y;
                    p.y = seg_start.y + s0 * r.x + c0 * r.y;
                    p.z = seg_start.z + 0.2;
                    line.points.push_back(p);
                }
                // Advance start to the end of this segment
                const auto& rlast = precomputed_rel_[ci].back();
                seg_start.x += c0 * rlast.x - s0 * rlast.y;
                seg_start.y += s0 * rlast.x + c0 * rlast.y;
                seg_start.heading += rlast.heading;
            }
        }

        msg.markers.push_back(line);
    }

    all_paths_pub_->publish(msg);
}


void path_planning::publishTrajectoryPath(const TreeFlat& flat, int leaf_idx)
{
    if (sdv_trajectory_pub_->get_subscription_count() == 0) return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    if (leaf_idx < 0) {
        sdv_trajectory_pub_->publish(path_msg);
        return;
    }

    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    // Helper to map (steer,dir) -> precomputed index
    auto find_cmd_index = [&](double steer, int dir)->int{
        for (size_t i = 0; i < motionCommand.size(); ++i)
            if (dir == (int)motionCommand[i][1] && std::abs(steer - motionCommand[i][0]) < 1e-9)
                return (int)i;
        return -1;
    };

    // Start at the root pose
    State seg_start = flat.nodes[chain.front()].state;
    
    // Add the starting point
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = path_msg.header.stamp;
    start_pose.pose.position.x = seg_start.x;
    start_pose.pose.position.y = seg_start.y;
    start_pose.pose.position.z = seg_start.z;
    
    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, seg_start.heading);
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();
    
    path_msg.poses.push_back(start_pose);

    // March along all segments root->leaf
    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[chain[k]];
        const int ci = find_cmd_index(fn.steer, fn.dir);

        if (ci >= 0 && ci < (int)precomputed_rel_.size() &&
            (int)precomputed_rel_[ci].size() == pathLength)
        {
            const double c0 = std::cos(seg_start.heading);
            const double s0 = std::sin(seg_start.heading);

            for (int i = 0; i < pathLength; ++i) {
                const auto& r = precomputed_rel_[ci][i];
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = path_msg.header.stamp;
                
                pose.pose.position.x = seg_start.x + c0 * r.x - s0 * r.y;
                pose.pose.position.y = seg_start.y + s0 * r.x + c0 * r.y;
                pose.pose.position.z = seg_start.z + 0.2;
                
                // Convert heading to quaternion
                double heading = seg_start.heading + r.heading;
                tf2::Quaternion q;
                q.setRPY(0, 0, heading);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
                
                path_msg.poses.push_back(pose);
            }
            
            // Advance start to the end of this segment
            const auto& rlast = precomputed_rel_[ci].back();
            seg_start.x += c0 * rlast.x - s0 * rlast.y;
            seg_start.y += s0 * rlast.x + c0 * rlast.y;
            seg_start.heading += rlast.heading;
        }
    }

    sdv_trajectory_pub_->publish(path_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}