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
    this->declare_parameter<int>("num_lateral_offsets", 0);

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
    this->get_parameter("num_lateral_offsets", num_lateral_offsets_);

    // subscription for ma comination btw the glonal ma and the obstacles information
    // occupancy_grid_complete_map_1 ->  map from occupancy_pub -> resolution 1.0
    // occupancy_grid_complete_map_2 -> map from osm visualizer -> resolution 0.2
    global_grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_complete_map_2", 10, std::bind(&path_planning::globalMap_callback, this, std::placeholders::_1));

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&path_planning::obstacle_info_callback, this, std::placeholders::_1));

    // publisher for the occupancy grid of the obstacles
    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);
    
    real_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_1", 10);

    real_trajectories_pub_2 = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_2", 10);

    global_planner_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/global_planner", 10);

    closest_waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/closest_waypoint_marker", 10);

    planner_waypoints_available_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner_waypoints_available", 10);

    planner_waypoint_polygons_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner_waypoint_vehicle_polygons", 10);
    
    // Continuous planning publishers
    planned_trajectories_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planned_trajectories", 10);
    optimal_trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/optimal_trajectory", 10);
    
    // Path joins publisher for Frenet frame trajectories
    path_joins_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_joins", 10);

    // -------------> Initialize the shared pointers  <------------
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    car_state_ = std::make_shared<State>();
    grid_map_ = nullptr;
    current_node = nullptr;
    global_planner_ = std::make_shared<GlobalPlanner>(x_offset_, y_offset_, map_path_, start_lanelet_id_, end_lanelet_id_);

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
    RCLCPP_INFO(this->get_logger(), "\033[1;34mnum_lateral_offsets: %d\033[0m", num_lateral_offsets_);

    // get the motion commans
    motionCommands();
    precomputeCommandSamples();
    all_waypoints_from_global_planner_ = global_planner_->getAllAllWaypointsStruct();
    publishGlobalPlanner();
    
    // Start continuous planning
    startContinuousPlanning();
}

path_planning::~path_planning()
{
    stopContinuousPlanning();
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

        compute_closest_waypoint();
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

// =============================
// get the closest waypoint to the car
// =============================
void path_planning::compute_closest_waypoint()
{
    if (all_waypoints_from_global_planner_.empty())
    {
        std::cout << red << "Warning: all_waypoints_from_global_planner_ is not available. Skipping close waypoint computation." << reset << std::endl;
        return;
    }

    const size_t total_waypoints = all_waypoints_from_global_planner_.size();
    double smallest_curr_distance = std::numeric_limits<double>::max();
    size_t new_closest_waypoint = closest_waypoint;

    // Optimization: Use hybrid search strategy
    // 1. First, search locally around the previous closest waypoint (fast path)
    // 2. Then, search all waypoints within a distance threshold (spatial filtering)
    // This handles both incremental movement and path switches efficiently
    
    constexpr int local_search_window = 30; // Increased window for local search
    constexpr double spatial_search_radius = 10.0; // meters - only check waypoints within this radius
    
    // Step 1: Local search around previous closest waypoint
    int search_start = std::max(0, static_cast<int>(closest_waypoint) - local_search_window);
    int search_end = std::min(static_cast<int>(total_waypoints) - 1, 
                             static_cast<int>(closest_waypoint) + local_search_window);
    
    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(all_waypoints_from_global_planner_[i]);
        if (curr_distance < smallest_curr_distance)
        {
            new_closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }
    
    // Step 2: Spatial search - check waypoints that are spatially close but might be far in the array
    // This is crucial for handling neighbor paths that are stored separately in the array
    // We use a coarse distance check first to avoid expensive exact distance calculations
    const double car_x = car_state_->x;
    const double car_y = car_state_->y;
    
    for (size_t i = 0; i < total_waypoints; i++)
    {
        // Skip if already checked in local search
        if (i >= static_cast<size_t>(search_start) && i <= static_cast<size_t>(search_end))
            continue;
        
        // Coarse distance check using Manhattan distance (faster than Euclidean)
        const double dx = std::abs(all_waypoints_from_global_planner_[i].x - car_x);
        const double dy = std::abs(all_waypoints_from_global_planner_[i].y - car_y);
        
        // Quick rejection: if Manhattan distance > radius * 1.5, skip
        if (dx + dy > spatial_search_radius * 1.5)
            continue;
        
        // Fine distance check for candidates
        double curr_distance = getDistanceFromOdom(all_waypoints_from_global_planner_[i]);
        
        // Only consider if within spatial radius
        if (curr_distance < spatial_search_radius && curr_distance < smallest_curr_distance)
        {
            new_closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }
    
    // Update closest waypoint
    closest_waypoint = new_closest_waypoint;
    
    // Log which lanelet the closest waypoint belongs to
    // int current_lanelet_id = all_waypoints_from_global_planner_[closest_waypoint].lanelet_id;
    // std::cout << green << "Closest waypoint: " << closest_waypoint 
    //           << " (lanelet_id: " << current_lanelet_id 
    //           << ", distance: " << smallest_curr_distance << " m)" << reset << std::endl;
    
    publish_closest_waypoint_marker();
}

double path_planning::getDistanceFromOdom(const point_struct& waypoint)
{
    double x1 = waypoint.x;
    double y1 = waypoint.y;
    double x2 = car_state_->x;
    double y2 = car_state_->y;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

// =============================
// publish the global planner
// =============================
void path_planning::publishGlobalPlanner()
{
    std::cout << green << "Publishing global planner" << reset << std::endl;
    std::cout << green << "Global planner size: " << all_waypoints_from_global_planner_.size() << reset << std::endl;
    
    // Debug: Print unique lanelet IDs for verification
    std::set<int> unique_lanelet_ids;
    for (const auto& waypoint : all_waypoints_from_global_planner_) {
        unique_lanelet_ids.insert(waypoint.lanelet_id);
    }
    std::cout << green << "Unique lanelet IDs: ";
    for (const auto& id : unique_lanelet_ids) {
        std::cout << id << " ";
    }
    std::cout << reset << std::endl;
    global_planner_markers_.markers.clear();
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
            
            // Color code based on lanelet_id for visual distinction
            int lanelet_id = all_waypoints_from_global_planner_[i].lanelet_id;
            
            // Use different colors based on lanelet ID
            if (lanelet_id == start_lanelet_id_) {
                // Start lanelet - Green
                waypoint_marker.color.r = 0.0;
                waypoint_marker.color.g = 1.0;
                waypoint_marker.color.b = 0.0;
            } else if (lanelet_id == end_lanelet_id_) {
                // End lanelet - Red
                waypoint_marker.color.r = 1.0;
                waypoint_marker.color.g = 0.0;
                waypoint_marker.color.b = 0.0;
            } else {
                // Other lanelets - Color based on lanelet ID hash
                // Use modulo to create consistent colors for each lanelet
                int color_hash = abs(lanelet_id) % 9; // Updated to 9 for the new colors
                switch (color_hash) {
                    case 0: // Blue
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;
                        break;
                    case 1: // Magenta
                        waypoint_marker.color.r = 1.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;
                        break;
                    case 2: // Cyan
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 1.0;
                        waypoint_marker.color.b = 1.0;
                        break;
                    case 3: // Yellow
                        waypoint_marker.color.r = 1.0;
                        waypoint_marker.color.g = 1.0;
                        waypoint_marker.color.b = 0.0;
                        break;
                    case 4: // Orange
                        waypoint_marker.color.r = 1.0;
                        waypoint_marker.color.g = 0.5;
                        waypoint_marker.color.b = 0.0;
                        break;
                    case 5: // Purple
                        waypoint_marker.color.r = 0.5;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;
                        break;
                    case 6: // Pink
                        waypoint_marker.color.r = 1.0;
                        waypoint_marker.color.g = 0.4;
                        waypoint_marker.color.b = 0.8;
                        break;
                    case 7: // 617T2REW4E GQSMIBYA5QRCBH,X
                        waypoint_marker.color.r = 0.5;
                        waypoint_marker.color.g = 0.25;
                        waypoint_marker.color.b = 0.0;
                        break;
                    case 8: // Gray
                        waypoint_marker.color.r = 0.5;
                        waypoint_marker.color.g = 0.5;
                        waypoint_marker.color.b = 0.5;
                        break;
                    default: // Default blue
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;
                        break;
                }
            }

            waypoint_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            waypoint_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            waypoint_marker.pose.position.z = -1.0;

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
        
        // Create text marker to display lanelet ID number
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "lanelet_id_text";
        text_marker.id = i;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position the text slightly above the arrow
        text_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
        text_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
        text_marker.pose.position.z = -0.5; // Higher than the arrow
        
        // Set text content to the lanelet ID
        text_marker.text = std::to_string(lanelet_id);
        
        // Text styling
        text_marker.scale.z = 0.3; // Text size
        text_marker.color.r = 1.0; // White text
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0; // Fully opaque
        
        global_planner_markers_.markers.push_back(text_marker);
    }
    global_planner_publisher_->publish(global_planner_markers_);
}

void path_planning::publish_closest_waypoint_marker()
{
    if (all_waypoints_from_global_planner_.empty() || closest_waypoint >= all_waypoints_from_global_planner_.size())
    {
        return;
    }

    // Create marker for the closest waypoint
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "closest_waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the position to the closest waypoint
    const auto& waypoint = all_waypoints_from_global_planner_[closest_waypoint];
    marker.pose.position.x = waypoint.x;
    marker.pose.position.y = waypoint.y;
    marker.pose.position.z = 0.0;

    // Set orientation (identity quaternion)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set scale (size of the sphere)
    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.9;

    // Set color (bright green for visibility)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Publish the marker
    closest_waypoint_marker_publisher_->publish(marker);
}

// =============================
// map combination & rescale for put obstacles in the global map
// =============================
void path_planning::globalMap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    global_map_ = map;
}

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

    // Iterate over the square and rotate points based on the heading
    for (int y = -half_square; y <= half_square; ++y)
    {
        for (int x = -half_square; x <= half_square; ++x)
        {
            // Rotate the point relative to the car's heading
            double rotated_x = x * cos_heading_white - y * sin_heading_white;
            double rotated_y = x * sin_heading_white + y * cos_heading_white    ;

            // Round and calculate final grid coordinates
            int square_x = car_x_rescaled + static_cast<int>(std::round(rotated_x));
            int square_y = car_y_rescaled + static_cast<int>(std::round(rotated_y));

            // Check bounds to avoid out-of-grid access
            if (square_x >= 0 && square_x < static_cast<int>(rescaled_chunk_->info.width) &&
                square_y >= 0 && square_y < static_cast<int>(rescaled_chunk_->info.height))
            {
                // Set to "white" (unknown)
                rescaled_chunk_->data[square_y * rescaled_chunk_->info.width + square_x] = 0;
            }
        }
    }

    buildDistanceField();
    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    grid_map_->setcarData(car_data_);

    // Publish the rescaled chunk
    if (occupancy_grid_pub_test_->get_subscription_count() > 0)
    {
        occupancy_grid_pub_test_->publish(*rescaled_chunk_);
    }

    // TreeFlat_global_planner flat_global_planner;
    // int best_global = generateTajectoyTree_from_global_planer(current_node->Current_state, flat_global_planner);
    // publish_planner_waypoints_available();
    // publish_planner_waypoint_polygons();

    TreeFlat flat;
    int best = generateTrajectoryTree_AStar_flat_map(current_node->Current_state, flat);
    publishBestPathFromFlat(flat, best, 1); // green color for the flat implementation

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

// =============================
// medium planner to get the path from the global planer
// =============================
int path_planning::generateTajectoyTree_from_global_planer(const State& root_state, TreeFlat_global_planner& out)
{
    out.nodes.clear();
    out.leaves.clear();

    planner_waypoints_available.clear();

    for (const auto& wp : all_waypoints_from_global_planner_)
    {
        State WaypointState_;
        WaypointState_.x = wp.x;
        WaypointState_.y = wp.y;
        WaypointState_.heading = wp.heading;

        // Check if the center waypoint is collision free
        bool collision = grid_map_->isSingleStateCollisionFreeImproved(WaypointState_);

        if (!collision)
        {
            planner_waypoints_available.push_back(wp);
        }

    }

    return 0;
}

void path_planning::publish_planner_waypoints_available()
{
    visualization_msgs::msg::MarkerArray msg;
    int marker_id = 0;
    for (const auto& wp : planner_waypoints_available)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "planner_waypoints_available";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wp.x;
        marker.pose.position.y = wp.y;
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        msg.markers.push_back(marker);
    }
    planner_waypoints_available_publisher_->publish(msg);
}


void path_planning::publish_planner_waypoint_polygons()
{
    if (planner_waypoint_polygons_publisher_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray msg;

    // clear previous markers in this namespace
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "planner_waypoint_vehicle_polygons"; msg.markers.push_back(clear);

    int id = 0;
    for (const auto& wp : planner_waypoints_available)
    {
        State s;
        s.x = wp.x;
        s.y = wp.y;
        s.z = 0.0;
        s.heading = wp.heading;

        geometry_msgs::msg::Polygon poly = car_data_.getVehicleGeometry_state(s);

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "planner_waypoint_vehicle_polygons";
        line_strip.id = id++;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.05; // line width
        line_strip.color.a = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 1.0;

        // convert polygon points into a closed line strip
        for (const auto& pt : poly.points)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.05;
            line_strip.points.push_back(p);
        }
        if (!poly.points.empty())
        {
            geometry_msgs::msg::Point p0;
            p0.x = poly.points.front().x;
            p0.y = poly.points.front().y;
            p0.z = 0.05;
            line_strip.points.push_back(p0);
        }

        msg.markers.push_back(line_strip);
    }

    planner_waypoint_polygons_publisher_->publish(msg);
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

// =============================
// Continuous Planning System
// =============================

void path_planning::startContinuousPlanning()
{
    if (continuous_planning_active_) {
        std::cout << yellow << "Continuous planning already active" << reset << std::endl;
        return;
    }
    
    continuous_planning_active_ = true;
    planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency_)),
        std::bind(&path_planning::continuousPlanningCallback, this)
    );
    
    std::cout << green << "Continuous planning started at " << planning_frequency_ << " Hz" << reset << std::endl;
}

void path_planning::stopContinuousPlanning()
{
    if (!continuous_planning_active_) {
        return;
    }
    
    continuous_planning_active_ = false;
    if (planning_timer_) {
        planning_timer_->cancel();
        planning_timer_.reset();
    }
    
    std::cout << red << "Continuous planning stopped" << reset << std::endl;
}

void path_planning::continuousPlanningCallback()
{
    // Update current robot state from TF (real odometry)
    getCurrentRobotState();
    
    // Generate multiple full trajectory options
    std::vector<std::vector<State>> all_trajectories = generateMultipleFullTrajectories(*car_state_);
    
    if (all_trajectories.empty()) {
        std::cout << red << "No valid trajectories generated" << reset << std::endl;
        return;
    }
    
    // Select optimal trajectory
    std::vector<State> optimal_trajectory = selectOptimalFullTrajectory(all_trajectories);
    
    // Visualize all trajectory options
    publishAllTrajectoriesVisualization(all_trajectories);
    
    // Publish optimal trajectory visualization
    publishTrajectoryVisualization(optimal_trajectory, "optimal_trajectory");
    
    // Create and publish Frenet frame joined paths
    createJoinedPaths();
    publishJoinedPaths();
    
    std::cout << blue << "Continuous planning cycle completed - Robot at: (" 
              << car_state_->x << ", " << car_state_->y << ", " << car_state_->heading << ")" << reset << std::endl;
}

std::vector<State> path_planning::generateTrajectory(const State& start_state, const State& target_state, double time_horizon)
{
    std::vector<State> trajectory;
    
    // Generate curved trajectory using polynomial interpolation (similar to Frenet frame)
    int num_steps = static_cast<int>(time_horizon / trajectory_time_step_);
    
    // Calculate distance and direction
    double dx = target_state.x - start_state.x;
    double dy = target_state.y - start_state.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // Initial and final velocities (similar to Frenet frame approach)
    double initial_velocity = 2.0; // m/s
    double final_velocity = 2.0;   // m/s
    
    // Calculate velocity components
    double vx0 = initial_velocity * std::cos(start_state.heading);
    double vy0 = initial_velocity * std::sin(start_state.heading);
    double vxf = final_velocity * std::cos(target_state.heading);
    double vyf = final_velocity * std::sin(target_state.heading);
    
    for (int i = 0; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        double T = time_horizon;
        
        State intermediate_state;
        
        // Use polynomial interpolation for smooth curves (similar to Frenet frame)
        // Position: cubic polynomial with boundary conditions
        double t2 = t * t;
        double t3 = t2 * t;
        
        // Cubic polynomial coefficients for smooth trajectory
        // P(t) = P0 + V0*t*T + (3*(Pf-P0)/T^2 - 2*V0/T - Vf/T)*t^2*T^2 + (2*(P0-Pf)/T^3 + (V0+Vf)/T^2)*t^3*T^3
        
        // X component
        double a0_x = start_state.x;
        double a1_x = vx0 * T;
        double a2_x = 3.0 * (target_state.x - start_state.x) - 2.0 * vx0 * T - vxf * T;
        double a3_x = 2.0 * (start_state.x - target_state.x) + (vx0 + vxf) * T;
        
        intermediate_state.x = a0_x + a1_x * t + a2_x * t2 + a3_x * t3;
        
        // Y component
        double a0_y = start_state.y;
        double a1_y = vy0 * T;
        double a2_y = 3.0 * (target_state.y - start_state.y) - 2.0 * vy0 * T - vyf * T;
        double a3_y = 2.0 * (start_state.y - target_state.y) + (vy0 + vyf) * T;
        
        intermediate_state.y = a0_y + a1_y * t + a2_y * t2 + a3_y * t3;
        
        // Calculate heading from velocity (derivative of position)
        double vx = (a1_x + 2.0 * a2_x * t + 3.0 * a3_x * t2) / T;
        double vy = (a1_y + 2.0 * a2_y * t + 3.0 * a3_y * t2) / T;
        
        if (std::abs(vx) > 0.01 || std::abs(vy) > 0.01) {
            intermediate_state.heading = std::atan2(vy, vx);
        } else {
            intermediate_state.heading = (i == 0) ? start_state.heading : trajectory.back().heading;
        }
        
        trajectory.push_back(intermediate_state);
    }
    
    return trajectory;
}

std::vector<State> path_planning::generateMultipleTrajectories(const State& start_state)
{
    std::vector<State> trajectory_options;
    
    if (all_waypoints_from_global_planner_.empty()) {
        std::cout << red << "No global waypoints available for trajectory generation" << reset << std::endl;
        return trajectory_options;
    }
    
    // Generate trajectories to different waypoints ahead of current position
    int start_idx = std::max(0, static_cast<int>(closest_waypoint));
    int end_idx = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                          static_cast<int>(closest_waypoint + 10));
    
    for (int i = start_idx + 1; i <= end_idx; i += 2) { // Sample every 2nd waypoint
        State target_state;
        target_state.x = all_waypoints_from_global_planner_[i].x;
        target_state.y = all_waypoints_from_global_planner_[i].y;
        target_state.heading = all_waypoints_from_global_planner_[i].heading;
        
        // Generate trajectory to this target
        std::vector<State> trajectory = generateTrajectory(start_state, target_state, max_trajectory_time_);
        
        if (!trajectory.empty()) {
            trajectory_options.push_back(trajectory.back()); // Store the final state as option
        }
    }
    
    return trajectory_options;
}

std::vector<std::vector<State>> path_planning::generateMultipleFullTrajectories(const State& start_state)
{
    std::vector<std::vector<State>> all_trajectories;
    
    if (all_waypoints_from_global_planner_.empty()) {
        std::cout << red << "No global waypoints available for trajectory generation" << reset << std::endl;
        return all_trajectories;
    }
    
    // Generate trajectories with different time horizons and lateral offsets (similar to Frenet frame)
    std::vector<double> time_horizons = {1.0, 1.5, 2.0, 2.5, 3.0}; // Different prediction times
    std::vector<double> lateral_offsets = generateLateralOffsets(); // Configurable lateral positions
    
    int start_idx = std::max(0, static_cast<int>(closest_waypoint));
    int end_idx = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                          static_cast<int>(closest_waypoint + 20));
    
    // Generate trajectories to different waypoints with various time horizons
    for (double T : time_horizons) {
        for (int i = start_idx + 1; i <= end_idx; i += 3) { // Sample every 3rd waypoint for variety
            State base_target;
            base_target.x = all_waypoints_from_global_planner_[i].x;
            base_target.y = all_waypoints_from_global_planner_[i].y;
            base_target.heading = all_waypoints_from_global_planner_[i].heading;
            
            // Create trajectory variations with lateral offsets
            for (double offset : lateral_offsets) {
                State target_state = base_target;
                
                // Apply lateral offset perpendicular to the waypoint heading
                target_state.x += offset * std::sin(base_target.heading);
                target_state.y -= offset * std::cos(base_target.heading);
                
                // Generate curved trajectory to this offset target
                std::vector<State> trajectory = generateTrajectory(start_state, target_state, T);
                
                if (!trajectory.empty()) {
                    all_trajectories.push_back(trajectory);
                }
            }
        }
    }
    
    // Add some trajectories that follow the global path more closely
    for (int i = start_idx + 1; i <= std::min(start_idx + 5, static_cast<int>(all_waypoints_from_global_planner_.size() - 1)); i++) {
        State target_state;
        target_state.x = all_waypoints_from_global_planner_[i].x;
        target_state.y = all_waypoints_from_global_planner_[i].y;
        target_state.heading = all_waypoints_from_global_planner_[i].heading;
        
        // Generate trajectory that closely follows the global path
        std::vector<State> trajectory = generateTrajectory(start_state, target_state, 2.0);
        
        if (!trajectory.empty()) {
            all_trajectories.push_back(trajectory);
        }
    }
    
    return all_trajectories;
}

std::vector<State> path_planning::selectOptimalFullTrajectory(const std::vector<std::vector<State>>& trajectories)
{
    if (trajectories.empty()) {
        return std::vector<State>(); // Return empty trajectory if no options
    }
    
    double best_cost = std::numeric_limits<double>::max();
    std::vector<State> optimal_trajectory = trajectories[0];
    
    for (const auto& trajectory : trajectories) {
        double cost = evaluateTrajectoryCost(trajectory);
        
        if (cost < best_cost) {
            best_cost = cost;
            optimal_trajectory = trajectory;
        }
    }
    
    std::cout << green << "Selected optimal full trajectory with cost: " << best_cost << reset << std::endl;
    return optimal_trajectory;
}

void path_planning::publishAllTrajectoriesVisualization(const std::vector<std::vector<State>>& trajectories)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();
    
    for (size_t traj_idx = 0; traj_idx < trajectories.size(); ++traj_idx) {
        const auto& trajectory = trajectories[traj_idx];
        
        // Create line strip marker for each trajectory path
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->now();
        line_marker.ns = "planned_trajectories";
        line_marker.id = static_cast<int>(traj_idx);
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Add points to the line strip
        for (size_t i = 0; i < trajectory.size(); ++i) {
            geometry_msgs::msg::Point point;
            point.x = trajectory[i].x;
            point.y = trajectory[i].y;
            point.z = 0.05; // Slightly lower than optimal trajectory
            line_marker.points.push_back(point);
        }
        
        // Set line properties
        line_marker.scale.x = 0.05; // Thinner lines for planned options
        line_marker.color.r = 0.0; // Blue for planned options
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.6; // More transparent
        
        marker_array.markers.push_back(line_marker);
    }
    
    planned_trajectories_publisher_->publish(marker_array);
}

State path_planning::selectOptimalTrajectory(const std::vector<State>& trajectories)
{
    if (trajectories.empty()) {
        return *car_state_; // Return current state if no options
    }
    
    double best_cost = std::numeric_limits<double>::max();
    State optimal_state = trajectories[0];
    
    for (const auto& trajectory_end : trajectories) {
        // Create a simple trajectory from current state to trajectory end
        std::vector<State> trajectory = generateTrajectory(*car_state_, trajectory_end, max_trajectory_time_);
        double cost = evaluateTrajectoryCost(trajectory);
        
        if (cost < best_cost) {
            best_cost = cost;
            optimal_state = trajectory_end;
        }
    }
    
    std::cout << green << "Selected optimal trajectory with cost: " << best_cost << reset << std::endl;
    return optimal_state;
}


void path_planning::publishTrajectoryVisualization(const std::vector<State>& trajectory, const std::string& namespace_name)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();
    
    // Create line strip marker for trajectory path
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = namespace_name;
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Add points to the line strip
    for (size_t i = 0; i < trajectory.size(); ++i) {
        geometry_msgs::msg::Point point;
        point.x = trajectory[i].x;
        point.y = trajectory[i].y;
        point.z = 0.1;
        line_marker.points.push_back(point);
    }
    
    // Set line properties
    line_marker.scale.x = 0.1; // Line width
    line_marker.color.a = 0.8;
    
    // Color based on namespace
    if (namespace_name == "optimal_trajectory") {
        line_marker.color.r = 1.0; // Red for optimal
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
    } else {
        line_marker.color.r = 0.0; // Blue for planned options
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
    }
    
    marker_array.markers.push_back(line_marker);
    
    // Add start point marker
    if (!trajectory.empty()) {
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = this->now();
        start_marker.ns = namespace_name + "_start";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        
        start_marker.pose.position.x = trajectory[0].x;
        start_marker.pose.position.y = trajectory[0].y;
        start_marker.pose.position.z = 0.2;
        
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        
        start_marker.color.r = 0.0; // Green for start
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        marker_array.markers.push_back(start_marker);
    }
    
    // Add end point marker
    if (trajectory.size() > 1) {
        visualization_msgs::msg::Marker end_marker;
        end_marker.header.frame_id = "map";
        end_marker.header.stamp = this->now();
        end_marker.ns = namespace_name + "_end";
        end_marker.id = 0;
        end_marker.type = visualization_msgs::msg::Marker::SPHERE;
        end_marker.action = visualization_msgs::msg::Marker::ADD;
        
        end_marker.pose.position.x = trajectory.back().x;
        end_marker.pose.position.y = trajectory.back().y;
        end_marker.pose.position.z = 0.2;
        
        end_marker.scale.x = 0.3;
        end_marker.scale.y = 0.3;
        end_marker.scale.z = 0.3;
        
        if (namespace_name == "optimal_trajectory") {
            end_marker.color.r = 1.0; // Red for optimal end
            end_marker.color.g = 0.0;
            end_marker.color.b = 0.0;
        } else {
            end_marker.color.r = 0.0; // Blue for planned end
            end_marker.color.g = 0.0;
            end_marker.color.b = 1.0;
        }
        end_marker.color.a = 1.0;
        
        marker_array.markers.push_back(end_marker);
    }
    
    // Publish based on namespace
    if (namespace_name == "optimal_trajectory") {
        optimal_trajectory_publisher_->publish(marker_array);
    } else {
        planned_trajectories_publisher_->publish(marker_array);
    }
}

double path_planning::evaluateTrajectoryCost(const std::vector<State>& trajectory)
{
    if (trajectory.size() < 2) {
        return std::numeric_limits<double>::max();
    }
    
    double total_cost = 0.0;
    
    // Distance cost (prefer shorter paths) - similar to Frenet frame
    double path_length = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - trajectory[i-1].x;
        double dy = trajectory[i].y - trajectory[i-1].y;
        path_length += std::sqrt(dx*dx + dy*dy);
    }
    total_cost += path_length * 0.1; // Distance weight
    
    // Smoothness cost (prefer smoother paths) - jerk minimization like Frenet
    double jerk_cost = 0.0;
    for (size_t i = 2; i < trajectory.size() - 1; ++i) {
        double curvature_change = std::abs(trajectory[i+1].heading - 2*trajectory[i].heading + trajectory[i-1].heading);
        jerk_cost += curvature_change * curvature_change;
    }
    total_cost += jerk_cost * 1.0; // Jerk weight
    
    // Velocity consistency cost
    double velocity_cost = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - trajectory[i-1].x;
        double dy = trajectory[i].y - trajectory[i-1].y;
        double velocity = std::sqrt(dx*dx + dy*dy) / trajectory_time_step_;
        
        // Penalize velocities outside desired range
        if (velocity > max_velocity_) {
            velocity_cost += (velocity - max_velocity_) * (velocity - max_velocity_);
        }
        if (velocity < 0.5) { // Minimum velocity
            velocity_cost += (0.5 - velocity) * (0.5 - velocity);
        }
    }
    total_cost += velocity_cost * 0.5; // Velocity weight
    
    // Goal alignment cost (prefer paths toward global waypoints)
    if (!all_waypoints_from_global_planner_.empty()) {
        // Find the best alignment with future waypoints
        double min_alignment_cost = std::numeric_limits<double>::max();
        
        int start_check = std::max(0, static_cast<int>(closest_waypoint));
        int end_check = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                               static_cast<int>(closest_waypoint + 5));
        
        for (int i = start_check; i <= end_check; ++i) {
            double dx = trajectory.back().x - all_waypoints_from_global_planner_[i].x;
            double dy = trajectory.back().y - all_waypoints_from_global_planner_[i].y;
            double alignment_cost = std::sqrt(dx*dx + dy*dy);
            min_alignment_cost = std::min(min_alignment_cost, alignment_cost);
        }
        
        total_cost += min_alignment_cost * 2.0; // Goal alignment weight
    }
    
    // Lane keeping cost (prefer staying close to global path)
    double lane_deviation_cost = 0.0;
    for (const auto& state : trajectory) {
        double min_distance_to_path = std::numeric_limits<double>::max();
        
        // Check distance to nearby waypoints
        int start_check = std::max(0, static_cast<int>(closest_waypoint) - 2);
        int end_check = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                               static_cast<int>(closest_waypoint + 10));
        
        for (int i = start_check; i <= end_check; ++i) {
            double dx = state.x - all_waypoints_from_global_planner_[i].x;
            double dy = state.y - all_waypoints_from_global_planner_[i].y;
            double distance = std::sqrt(dx*dx + dy*dy);
            min_distance_to_path = std::min(min_distance_to_path, distance);
        }
        
        lane_deviation_cost += min_distance_to_path;
    }
    total_cost += lane_deviation_cost * 0.3; // Lane keeping weight
    
    return total_cost;
}

std::vector<double> path_planning::generateLateralOffsets()
{
    std::vector<double> lateral_offsets;
    
    if (num_lateral_offsets_ == 0) {
        // Only center path
        lateral_offsets.push_back(0.0);
    } else {
        // Generate symmetric offsets around center
        double offset_step = 0.75; // 0.75m increments
        
        // Always include center
        lateral_offsets.push_back(0.0);
        
        // Add symmetric left and right offsets
        for (int i = 1; i <= num_lateral_offsets_; ++i) {
            double offset = i * offset_step;
            lateral_offsets.push_back(-offset); // Left (negative)
            lateral_offsets.push_back(offset);  // Right (positive)
        }
    }
    
    return lateral_offsets;
}

// =============================
// Frenet Frame Path Joining Methods
// =============================

void path_planning::createJoinedPaths()
{
    if (all_waypoints_from_global_planner_.empty()) {
        std::cout << red << "No global waypoints available for path joining" << reset << std::endl;
        return;
    }
    
    // Group waypoints by lanelet
    std::vector<std::vector<point_struct>> lanelet_groups = groupWaypointsByLanelet();
    
    // Create longitudinal path (main path following)
    longitudinal_path_ = createLongitudinalPath();
    
    // Create lateral paths (joining different lanelets)
    lateral_paths_ = createLateralPaths();
    
    std::cout << green << "Created " << longitudinal_path_.size() << " longitudinal waypoints and " 
              << lateral_paths_.size() << " lateral paths" << reset << std::endl;
    
    // Debug: Print lanelet transitions for longitudinal path
    if (!longitudinal_path_.empty()) {
        std::cout << blue << "Longitudinal path lanelet transitions: ";
        int prev_lanelet = longitudinal_path_[0].lanelet_id;
        std::cout << prev_lanelet;
        for (size_t i = 1; i < longitudinal_path_.size(); i++) {
            if (longitudinal_path_[i].lanelet_id != prev_lanelet) {
                std::cout << " -> " << longitudinal_path_[i].lanelet_id;
                prev_lanelet = longitudinal_path_[i].lanelet_id;
            }
        }
        std::cout << reset << std::endl;
    }
    
    // Debug: Print lanelet transitions for lateral paths
    for (size_t i = 0; i < lateral_paths_.size(); ++i) {
        if (!lateral_paths_[i].empty()) {
            std::cout << blue << "Lateral path " << i << " lanelet transitions: ";
            int prev_lanelet = lateral_paths_[i][0].lanelet_id;
            std::cout << prev_lanelet;
            for (size_t j = 1; j < lateral_paths_[i].size(); j++) {
                if (lateral_paths_[i][j].lanelet_id != prev_lanelet) {
                    std::cout << " -> " << lateral_paths_[i][j].lanelet_id;
                    prev_lanelet = lateral_paths_[i][j].lanelet_id;
                }
            }
            std::cout << reset << std::endl;
        }
    }
}

std::vector<std::vector<point_struct>> path_planning::groupWaypointsByLanelet()
{
    std::map<int, std::vector<point_struct>> lanelet_map;
    
    // Group waypoints by lanelet_id
    for (const auto& waypoint : all_waypoints_from_global_planner_) {
        lanelet_map[waypoint.lanelet_id].push_back(waypoint);
    }
    
    // Convert map to vector of vectors
    std::vector<std::vector<point_struct>> groups;
    for (const auto& pair : lanelet_map) {
        groups.push_back(pair.second);
    }
    
    // Sort groups by lanelet_id for consistent ordering
    std::sort(groups.begin(), groups.end(), [](const std::vector<point_struct>& a, const std::vector<point_struct>& b) {
        return !a.empty() && !b.empty() && a[0].lanelet_id < b[0].lanelet_id;
    });
    
    return groups;
}

std::vector<point_struct> path_planning::createLongitudinalPath()
{
    std::vector<point_struct> longitudinal_path;
    
    // Create a continuous longitudinal path that smoothly transitions between lanelets
    int start_idx = std::max(0, static_cast<int>(closest_waypoint));
    int end_idx = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                          static_cast<int>(closest_waypoint + 25));
    
    // Strategy: Follow consecutive waypoints forward, allowing smooth lanelet transitions
    // This creates a continuous path that follows the main route regardless of lanelet changes
    
    for (int i = start_idx; i <= end_idx; i++) {
        const auto& waypoint = all_waypoints_from_global_planner_[i];
        
        // Always include the waypoint to maintain continuity
        longitudinal_path.push_back(waypoint);
        
        // Optional: Add some filtering based on spatial continuity
        if (longitudinal_path.size() > 1) {
            const auto& last_waypoint = longitudinal_path[longitudinal_path.size() - 2];
            double dx = waypoint.x - last_waypoint.x;
            double dy = waypoint.y - last_waypoint.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            // If the distance is too large, it might be a disconnected waypoint
            // Keep it anyway for now, but this could be used for filtering if needed
            if (distance > 10.0) { // More than 10 meters apart
                // Still keep it, but could add logic here if needed
            }
        }
    }
    
    return longitudinal_path;
}

std::vector<std::vector<point_struct>> path_planning::createLateralPaths()
{
    std::vector<std::vector<point_struct>> lateral_paths;
    
    // Get unique lanelet IDs and their waypoints
    std::map<int, std::vector<point_struct>> lanelet_map;
    for (const auto& waypoint : all_waypoints_from_global_planner_) {
        lanelet_map[waypoint.lanelet_id].push_back(waypoint);
    }
    
    // Find the current lanelet based on closest waypoint
    int current_lanelet_id = all_waypoints_from_global_planner_[closest_waypoint].lanelet_id;
    
    // Create continuous lateral paths for each different lanelet
    for (const auto& lanelet_pair : lanelet_map) {
        int target_lanelet_id = lanelet_pair.first;
        if (target_lanelet_id == current_lanelet_id) continue; // Skip current lanelet
        
        std::vector<point_struct> lateral_path;
        
        // Strategy: Create a continuous path that includes waypoints from the target lanelet
        // and connects them smoothly, even across lanelet boundaries
        int start_idx = std::max(0, static_cast<int>(closest_waypoint));
        int end_idx = std::min(static_cast<int>(all_waypoints_from_global_planner_.size() - 1), 
                              static_cast<int>(closest_waypoint + 20));
        
        // First pass: Collect waypoints from the target lanelet
        std::vector<point_struct> target_lanelet_waypoints;
        for (int i = start_idx; i <= end_idx; i++) {
            if (all_waypoints_from_global_planner_[i].lanelet_id == target_lanelet_id) {
                target_lanelet_waypoints.push_back(all_waypoints_from_global_planner_[i]);
            }
        }
        
        // Second pass: Create continuous path by connecting target lanelet waypoints
        // and filling gaps with spatially close waypoints from other lanelets
        for (int i = start_idx; i <= end_idx; i++) {
            const auto& waypoint = all_waypoints_from_global_planner_[i];
            
            // Always include waypoints from the target lanelet
            if (waypoint.lanelet_id == target_lanelet_id) {
                lateral_path.push_back(waypoint);
            }
            // Include waypoints from other lanelets if they help maintain continuity
            else if (!lateral_path.empty()) {
                const auto& last_waypoint = lateral_path.back();
                double dx = waypoint.x - last_waypoint.x;
                double dy = waypoint.y - last_waypoint.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                // If this waypoint is close to the last one and helps maintain continuity
                if (distance < 3.0) { // Within 3 meters
                    // Check if it's moving in a reasonable direction
                    double forward_progress = (waypoint.x - last_waypoint.x) * std::cos(car_state_->heading) + 
                                            (waypoint.y - last_waypoint.y) * std::sin(car_state_->heading);
                    
                    // Allow some backward movement for continuity, but prefer forward
                    if (forward_progress > -1.0) { // Allow slight backward movement
                        lateral_path.push_back(waypoint);
                    }
                }
            }
        }
        
        // If we still don't have enough waypoints, try spatial proximity approach
        if (lateral_path.size() < 3) {
            lateral_path.clear();
            
            // Find waypoints from target lanelet that are spatially close to longitudinal path
            for (const auto& waypoint : lanelet_pair.second) {
                bool is_close = false;
                for (const auto& long_waypoint : longitudinal_path_) {
                    double dx = waypoint.x - long_waypoint.x;
                    double dy = waypoint.y - long_waypoint.y;
                    double distance = std::sqrt(dx*dx + dy*dy);
                    
                    if (distance < 6.0) { // Within 6 meters
                        is_close = true;
                        break;
                    }
                }
                
                if (is_close) {
                    lateral_path.push_back(waypoint);
                }
            }
            
            // Sort by distance from current position
            std::sort(lateral_path.begin(), lateral_path.end(), 
                      [this](const point_struct& a, const point_struct& b) {
                          double dist_a = getDistanceFromOdom(a);
                          double dist_b = getDistanceFromOdom(b);
                          return dist_a < dist_b;
                      });
        }
        
        // Only add if we have at least 2 waypoints for a meaningful path
        if (lateral_path.size() >= 2) {
            lateral_paths.push_back(lateral_path);
        }
    }
    
    return lateral_paths;
}

void path_planning::publishJoinedPaths()
{
    if (path_joins_publisher_->get_subscription_count() == 0) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "path_joins";
    marker_array.markers.push_back(clear);
    
    int marker_id = 0;
    
    // Publish longitudinal path (main path) - GREEN
    if (!longitudinal_path_.empty()) {
        visualization_msgs::msg::Marker longitudinal_marker;
        longitudinal_marker.header.frame_id = "map";
        longitudinal_marker.header.stamp = this->now();
        longitudinal_marker.ns = "path_joins";
        longitudinal_marker.id = marker_id++;
        longitudinal_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        longitudinal_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Add points to the line strip
        for (const auto& waypoint : longitudinal_path_) {
            geometry_msgs::msg::Point point;
            point.x = waypoint.x;
            point.y = waypoint.y;
            point.z = 0.1; // Slightly elevated
            longitudinal_marker.points.push_back(point);
        }
        
        // Set line properties - GREEN for longitudinal path
        longitudinal_marker.scale.x = 0.15; // Thicker line for main path
        longitudinal_marker.color.r = 0.0;
        longitudinal_marker.color.g = 1.0;
        longitudinal_marker.color.b = 0.0;
        longitudinal_marker.color.a = 1.0;
        
        marker_array.markers.push_back(longitudinal_marker);
    }
    
    // Publish lateral paths - Different colors for each path
    std::vector<std::vector<double>> colors = {
        {1.0, 0.0, 0.0}, // Red
        {0.0, 0.0, 1.0}, // Blue
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}, // Cyan
        {1.0, 1.0, 0.0}, // Yellow
        {1.0, 0.5, 0.0}, // Orange
        {0.5, 0.0, 1.0}, // Purple
        {1.0, 0.4, 0.8}  // Pink
    };
    
    for (size_t i = 0; i < lateral_paths_.size(); ++i) {
        const auto& lateral_path = lateral_paths_[i];
        
        visualization_msgs::msg::Marker lateral_marker;
        lateral_marker.header.frame_id = "map";
        lateral_marker.header.stamp = this->now();
        lateral_marker.ns = "path_joins";
        lateral_marker.id = marker_id++;
        lateral_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lateral_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Add points to the line strip
        for (const auto& waypoint : lateral_path) {
            geometry_msgs::msg::Point point;
            point.x = waypoint.x;
            point.y = waypoint.y;
            point.z = 0.05; // Slightly lower than longitudinal path
            lateral_marker.points.push_back(point);
        }
        
        // Set line properties - Different color for each lateral path
        lateral_marker.scale.x = 0.1; // Thinner lines for lateral paths
        size_t color_idx = i % colors.size();
        lateral_marker.color.r = colors[color_idx][0];
        lateral_marker.color.g = colors[color_idx][1];
        lateral_marker.color.b = colors[color_idx][2];
        lateral_marker.color.a = 0.8;
        
        marker_array.markers.push_back(lateral_marker);
        
        // Add waypoint markers for lateral paths
        for (size_t j = 0; j < lateral_path.size(); j += 3) { // Every 3rd waypoint
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "path_joins_waypoints";
            waypoint_marker.id = marker_id++;
            waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            
            waypoint_marker.pose.position.x = lateral_path[j].x;
            waypoint_marker.pose.position.y = lateral_path[j].y;
            waypoint_marker.pose.position.z = 0.1;
            
            waypoint_marker.scale.x = 0.2;
            waypoint_marker.scale.y = 0.2;
            waypoint_marker.scale.z = 0.2;
            
            waypoint_marker.color.r = colors[color_idx][0];
            waypoint_marker.color.g = colors[color_idx][1];
            waypoint_marker.color.b = colors[color_idx][2];
            waypoint_marker.color.a = 0.6;
            
            marker_array.markers.push_back(waypoint_marker);
        }
    }
    
    // Add waypoint markers for longitudinal path with lanelet transition indicators
    int prev_lanelet_id = -1;
    for (size_t i = 0; i < longitudinal_path_.size(); i += 2) { // Every 2nd waypoint
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = "map";
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "path_joins_waypoints";
        waypoint_marker.id = marker_id++;
        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        
        waypoint_marker.pose.position.x = longitudinal_path_[i].x;
        waypoint_marker.pose.position.y = longitudinal_path_[i].y;
        waypoint_marker.pose.position.z = 0.15;
        
        waypoint_marker.scale.x = 0.25; // Larger for main path
        waypoint_marker.scale.y = 0.25;
        waypoint_marker.scale.z = 0.25;
        
        // Color based on lanelet transitions
        if (longitudinal_path_[i].lanelet_id != prev_lanelet_id) {
            // Lanelet transition - use brighter color
            waypoint_marker.color.r = 0.0; // Green
            waypoint_marker.color.g = 1.0;
            waypoint_marker.color.b = 0.0;
            waypoint_marker.scale.x = 0.35; // Even larger for transitions
            waypoint_marker.scale.y = 0.35;
            waypoint_marker.scale.z = 0.35;
        } else {
            // Same lanelet - use normal color
            waypoint_marker.color.r = 0.0; // Green
            waypoint_marker.color.g = 0.8;
            waypoint_marker.color.b = 0.0;
        }
        waypoint_marker.color.a = 0.8;
        
        marker_array.markers.push_back(waypoint_marker);
        prev_lanelet_id = longitudinal_path_[i].lanelet_id;
        
        // Add text marker for lanelet transitions
        if (i == 0 || longitudinal_path_[i].lanelet_id != prev_lanelet_id) {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "path_joins_lanelet_labels";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = longitudinal_path_[i].x;
            text_marker.pose.position.y = longitudinal_path_[i].y;
            text_marker.pose.position.z = 0.3;
            
            text_marker.text = "L" + std::to_string(longitudinal_path_[i].lanelet_id);
            text_marker.scale.z = 0.2;
            text_marker.color.r = 0.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
            
            marker_array.markers.push_back(text_marker);
        }
    }
    
    path_joins_publisher_->publish(marker_array);
    
    std::cout << blue << "Published " << marker_array.markers.size() << " path join markers" << reset << std::endl;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}