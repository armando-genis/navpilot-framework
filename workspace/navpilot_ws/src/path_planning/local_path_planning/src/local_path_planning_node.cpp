#include "local_path_planning_node.h"

// to do list
// the fist points ahead the car be 0
// see  ig the crosswalks are correct

local_path_planning_node::local_path_planning_node(/* args */) : Node("local_path_planning_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<double>("pathLength", 0.0);
    this->declare_parameter<double>("step_car", 0.0);

    this->get_parameter("maxSteerAngle", maxSteerAngle);
    this->get_parameter("wheelBase", wheelBase);
    this->get_parameter("axleToFront", axleToFront);
    this->get_parameter("axleToBack", axleToBack);
    this->get_parameter("width", width);
    this->get_parameter("pathLength", pathLength);
    this->get_parameter("step_car", step_car);

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&local_path_planning_node::obstacle_info_callback, this, std::placeholders::_1));

    road_elements_subscription_ = this->create_subscription<traffic_information_msgs::msg::RoadElementsCollection>(
        "/road_elements", 10, std::bind(&local_path_planning_node::roadElementsCallback, this, std::placeholders::_1));

    yaw_car_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/sdv/steering/position", 10, std::bind(&local_path_planning_node::yawCarCallback, this, std::placeholders::_1));

    waypoints_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/waypoints_routing", 10, std::bind(&local_path_planning_node::waypoints_callback, this, std::placeholders::_1));

    global_grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_complete_map", 10, std::bind(&local_path_planning_node::globalMap_callback, this, std::placeholders::_1));

    // ------------> Publishers <------------

    lane_steering_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "steering_lane", 10);

    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_waypoints", 10);

    // path_publisher_real = this->create_publisher<nav_msgs::msg::Path>("path_to_follow", 10);

    gear_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/gear_selection", rclcpp::SystemDefaultsQoS());

    percentage_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/percentage", rclcpp::SystemDefaultsQoS());

    // -------------> Publishers for path  <------------

    car_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/car_path", 10);

    emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/sdv/emergency_stop", 10);

    closest_waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "closest_waypoint_marker", 10);

    crosswalk_polygon_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "crosswalk_polygon_markers", 10);

    // crosswalk_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     "crosswalk_markers", 10);

    // -------------> Initialize the shared pointers  <------------
    vehicle_path = std::make_shared<geometry_msgs::msg::Polygon>();
    segment_path = std::make_shared<geometry_msgs::msg::Polygon>();
    collision_vector = std::make_shared<std::vector<bool>>();
    waypoints = std::make_shared<vector<Eigen::VectorXd>>();
    car_state_ = std::make_shared<State>();
    waypoints_segmentation = std::make_shared<vector<Eigen::VectorXd>>();
    waypoints_historical = std::make_shared<vector<Eigen::VectorXd>>();
    optimal_path = std::make_shared<vector<Eigen::VectorXd>>();
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    // Create the vehicle geometry
    car_data_ = CarData(maxSteerAngle, wheelBase, axleToFront, axleToBack, width);
    car_data_.createVehicleGeometry();

    motionCommands();

    // log out parameters in blue color
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmaxSteerAngle: %f\033[0m", maxSteerAngle);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwheelBase: %f\033[0m", wheelBase);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToFront: %f\033[0m", axleToFront);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToBack: %f\033[0m", axleToBack);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwidth: %f\033[0m", width);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %f\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> local_path_planning_node initialized.\033[0m");
}

local_path_planning_node::~local_path_planning_node()
{
}

void local_path_planning_node::motionCommands()
{
    int direction = 1;
    double steep = car_data_.maxSteerAngle / 2;

    for (double i = car_data_.maxSteerAngle; i >= -car_data_.maxSteerAngle; i -= steep)
    {
        // fist data: steering angle, second data: direction
        motionCommand.push_back({i, static_cast<double>(direction)});
    }
}

void local_path_planning_node::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        car_state_->x = pose_tf.translation.x;
        car_state_->y = pose_tf.translation.y;
        car_state_->z = pose_tf.translation.z - 1.55;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_->heading = yaw;

        compute_closest_waypoint();
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

void local_path_planning_node::crosswalk_visited_check(const traffic_information_msgs::msg::RoadElements &crosswalk)
{
    double center_x = 0.0;
    double center_y = 0.0;
    int num_points = crosswalk.points.size();

    for (const auto &point : crosswalk.points)
    {
        center_x += point.x;
        center_y += point.y;
    }
    center_x /= num_points;
    center_y /= num_points;

    // Check for collision only if the crosswalk is near the the car (e.g., within 3 meters)
    // if (std::abs(crosswalk.points[0].x - car_state_->x) < 5 && std::abs(crosswalk.points[0].y - car_state_->y) < 5)
    if (std::abs(center_x - car_state_->x) < 5 && std::abs(center_y - car_state_->y) < 5)
    {
        // append the element id to the list of visited crosswalks
        skip_ids.push_back(crosswalk.id);
        segment_start_index_temp = segment_start_index;
    }
}

void local_path_planning_node::compute_closest_waypoint()
{

    if (waypoints->empty())
    {
        std::cout << red << "Warning: waypoints is not available. Skipping close waypoint computation." << reset << std::endl;
        return;
    }

    constexpr size_t first_wp = 0;
    const size_t last_wp = waypoints->size() - 1;
    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double smallest_curr_distance = std::numeric_limits<double>::max();

    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints->at(i));
        if (smallest_curr_distance > curr_distance)
        {
            closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Closest waypoint: %d", closest_waypoint);

    publish_closest_waypoint_marker();
}

void local_path_planning_node::publish_closest_waypoint_marker()
{
    if (waypoints->empty() || closest_waypoint >= waypoints->size())
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
    const auto& waypoint = waypoints->at(closest_waypoint);
    marker.pose.position.x = waypoint(0);
    marker.pose.position.y = waypoint(1);
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

void local_path_planning_node::compute_path_polygon()
{

    if (waypoints_segmentation->empty())
    {
        std::cout << red << "Warning: waypoints segmentation is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    segment_path->points.clear();

    // Prepare Marker for visualization
    visualization_msgs::msg::Marker lane_maker;
    lane_maker.header.frame_id = "map";
    lane_maker.header.stamp = this->now();

    lane_maker.ns = "vehicle_path";
    lane_maker.id = 0;
    lane_maker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_maker.action = visualization_msgs::msg::Marker::ADD;
    lane_maker.scale.x = 0.07;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5;
    lane_maker.color.a = 1.0;

    lane_maker.color.r = 1.0;
    lane_maker.color.g = 0.0;
    lane_maker.color.b = 0.0;

    // Publish lane with the track of the path for the left side
    for (size_t i = 0; i < waypoints_segmentation->size(); ++i)
    {
        double angle = waypoints_segmentation->at(i)(3);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = waypoints_segmentation->at(i)(0) + offset_x;
        converted_point.y = waypoints_segmentation->at(i)(1) + offset_y;
        converted_point.z = 0.0;
        segment_path->points.push_back(converted_point);

        geometry_msgs::msg::Point left_point;
        left_point.x = converted_point.x;
        left_point.y = converted_point.y;
        left_point.z = 0.0;
        lane_maker.points.push_back(left_point);
    }

    //  invertion of the right side to math with the left side
    for (size_t i = waypoints_segmentation->size(); i-- > 0;)
    {
        double angle = waypoints_segmentation->at(i)(3);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = waypoints_segmentation->at(i)(0) - offset_x;
        converted_point.y = waypoints_segmentation->at(i)(1) - offset_y;
        converted_point.z = 0.0;
        segment_path->points.push_back(converted_point);

        geometry_msgs::msg::Point right_point;
        right_point.x = converted_point.x;
        right_point.y = converted_point.y;
        right_point.z = 0.0;
        lane_maker.points.push_back(right_point);
    }

    segment_path->points.push_back(segment_path->points.front());
    lane_maker.points.push_back(lane_maker.points.front());
    // lane_steering_publisher_->publish(lane_maker);

    // if (lane_steering_publisher_->get_subscription_count() > 0)
    // {
    //     lane_steering_publisher_->publish(lane_maker);
    // }
}

double
local_path_planning_node::getDistanceFromOdom(Eigen::VectorXd wapointPoint)
{
    double x1 = wapointPoint(0);
    double y1 = wapointPoint(1);
    double x2 = car_state_->x;
    double y2 = car_state_->y;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void local_path_planning_node::sampleLayersAlongPath()
{

    if (waypoints_segmentation->empty())
    {
        std::cout << red << "Warning: waypoints segmentation is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    if (rescaled_chunk_->data.empty())
    {
        std::cout << red << "Warning: combined map is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    // print the leng of the waypoints_segmentation 
    std::cout << yellow << "waypoints_segmentation size: " << waypoints_segmentation->size() << reset << std::endl;

    // MarkerArray to hold all markers
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    grid_map_->setcarData(car_data_);
    std::vector<std::vector<Eigen::VectorXd>> layers_samples;

    // Vector to store accumulated costs for each sample point
    std::vector<std::vector<double>> accumulated_costs;
    std::vector<std::vector<int>> predecessors;

    for (size_t layer_idx = 0; layer_idx < waypoints_segmentation->size(); ++layer_idx)
    {
        const auto &waypoint = waypoints_segmentation->at(layer_idx);
        double x = waypoint(0);
        double y = waypoint(1);
        double yaw = waypoint(3);
        std::vector<Eigen::VectorXd> layer_points;
        std::vector<double> layer_costs;
        std::vector<int> layer_predecessors;

        // for the center waypoint
        State WaypointState_;
        WaypointState_.x = x;
        WaypointState_.y = y;
        WaypointState_.heading = yaw;

        Eigen::VectorXd center_point(3);
        center_point(0) = x;
        center_point(1) = y;
        center_point(2) = yaw;

        // Check if the center waypoint is collision free
        bool collision = grid_map_->isSingleStateCollisionFreeImproved(WaypointState_);

        if (!collision)
        {

            layer_points.push_back(center_point);

            // Calculate cost for the waypoint
            double min_cost = std::numeric_limits<double>::max();
            int best_predecessor = -1;

            if (!accumulated_costs.empty())
            {
                const auto &previous_layer_points = layers_samples.back();
                const auto &previous_layer_costs = accumulated_costs.back();

                for (size_t prev_idx = 0; prev_idx < previous_layer_points.size(); ++prev_idx)
                {
                    double cost = computeCost(center_point, previous_layer_points[prev_idx], layer_idx) + previous_layer_costs[prev_idx];
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        best_predecessor = prev_idx;
                    }
                }
            }
            else
            {
                min_cost = 0.0; // Initial layer has no predecessor
            }

            if (layer_idx < start_waypoint_idx)
            {
                min_cost = 0.0;
            }

            layer_costs.push_back(min_cost);
            layer_predecessors.push_back(best_predecessor);
        }

        if (layer_idx >= start_waypoint_idx)
        {
            // for the lateral offset
            for (double lateral_offset = -lateral_range; lateral_offset <= lateral_range; lateral_offset += lateral_spacing)
            {
                Eigen::VectorXd sample_point(3);
                sample_point(0) = x + lateral_offset * cos(yaw + M_PI_2);
                sample_point(1) = y + lateral_offset * sin(yaw + M_PI_2);
                sample_point(2) = yaw;

                Eigen::Vector2d pos(sample_point(0), sample_point(1));

                State CurrentState_;
                CurrentState_.x = sample_point(0);
                CurrentState_.y = sample_point(1);
                CurrentState_.heading = sample_point(2);

                // double obstacle_distance = grid_map_->getObstacleDistance(pos);
                bool collision = grid_map_->isSingleStateCollisionFreeImproved(CurrentState_);

                // if (obstacle_distance > search_threshold)
                if (!collision)
                {
                    layer_points.push_back(sample_point);

                    // Calculate cost to this point from previous layer
                    double min_cost = std::numeric_limits<double>::max();
                    int best_predecessor = -1;

                    if (!accumulated_costs.empty())
                    {
                        const auto &previous_layer_points = layers_samples.back();
                        const auto &previous_layer_costs = accumulated_costs.back();

                        for (size_t prev_idx = 0; prev_idx < previous_layer_points.size(); ++prev_idx)
                        {
                            double cost = computeCost(sample_point, previous_layer_points[prev_idx], layer_idx) + previous_layer_costs[prev_idx];
                            if (cost < min_cost)
                            {
                                min_cost = cost;
                                best_predecessor = prev_idx;
                            }
                        }
                    }
                    else
                    {
                        min_cost = 0.0; // Initial layer has no predecessor
                    }

                    // if (layer_idx < 4)
                    // {
                    //     min_cost = 10000.0;
                    // }

                    layer_costs.push_back(min_cost);
                    layer_predecessors.push_back(best_predecessor);
                }
            }
        }

        // If there are points in the layer, add them to the list
        if (layer_points.size() > 0)
        {
            // Save layer information for later path extraction
            layers_samples.push_back(layer_points);
            accumulated_costs.push_back(layer_costs);
            predecessors.push_back(layer_predecessors);
        }
    }

    // Retrieve optimal path by backtracking from the final layer
    optimal_path->clear();
    if (!accumulated_costs.empty())
    {
        double min_final_cost = std::numeric_limits<double>::max();
        int best_final_index = -1;

        // Find the point in the last layer with the minimum cost
        for (size_t i = 0; i < accumulated_costs.back().size(); ++i)
        {
            if (accumulated_costs.back()[i] < min_final_cost)
            {
                min_final_cost = accumulated_costs.back()[i];
                best_final_index = i;
            }
        }

        // Backtrack to reconstruct the optimal path
        int layer_idx = accumulated_costs.size() - 1;
        while (layer_idx >= 0 && best_final_index >= 0)
        {
            optimal_path->push_back(layers_samples[layer_idx][best_final_index]);
            best_final_index = predecessors[layer_idx][best_final_index];
            --layer_idx;
        }

        std::reverse(optimal_path->begin(), optimal_path->end());
    }

    computeSplitpath();
}

void local_path_planning_node::computeSplitpath()
{
    // Static variable to track previous marker IDs for cleanup
    static std::set<int> previous_marker_ids;
    
    // Function to clear all previous markers
    auto clearAllMarkers = [&]() {
        visualization_msgs::msg::MarkerArray clear_array;
        for (int previous_id : previous_marker_ids)
        {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = this->now();
            delete_marker.ns = "next_state_polygons";
            delete_marker.id = previous_id;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            clear_array.markers.push_back(delete_marker);
        }
        if (!clear_array.markers.empty())
        {
            car_path_pub_->publish(clear_array);
        }
        previous_marker_ids.clear();
    };

    if (optimal_path->empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Optimal path is empty.");
        clearAllMarkers();
        return;
    }

    if (optimal_path->size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "Insufficient points in optimal path for spline interpolation.");
        clearAllMarkers();
        return;
    }

    std::cout << yellow << "num of optimal path points: " << optimal_path->size() << reset << std::endl;

    double max_angle = M_PI / 6;
    std::vector<tinyspline::real> filtered_control_points;

    for (size_t i = 1; i < optimal_path->size() - 1; ++i)
    {
        const auto &prev = (*optimal_path)[i - 1];
        const auto &current = (*optimal_path)[i];
        const auto &next = (*optimal_path)[i + 1];

        double angle1 = atan2(current(1) - prev(1), current(0) - prev(0));
        double angle2 = atan2(next(1) - current(1), next(0) - current(0));

        double angle_diff = fabs(angle2 - angle1);
        if (angle_diff > max_angle)
        {
            continue;
        }

        filtered_control_points.push_back(current(0));
        filtered_control_points.push_back(current(1));
    }

    int degree = std::min(3, static_cast<int>(filtered_control_points.size() / 2) - 1);

    if (degree < 1)
    {
        RCLCPP_WARN(this->get_logger(), "Calculated spline degree is invalid.");
        clearAllMarkers();
        return;
    }
    tinyspline::BSpline spline(filtered_control_points.size() / 2, 2, degree);
    spline.setControlPoints(filtered_control_points);

    const size_t num_samples = 100;
    std::vector<tinyspline::real> result = spline.sample(num_samples);

    double distance = sqrt(pow(result[0] - car_state_->x, 2) + pow(result[1] - car_state_->y, 2));
    if (distance > 3.3)
    {
        std::cout << red << "---> Distance between the car and the first waypoint of the path is greater than 3.3m." << reset << std::endl;
        clearAllMarkers();
        return;
    }

    if (collision_detected_trajectory)
    {
        std::cout << red << "---> obstacle near the car. Skip creating a path" << reset << std::endl;
        clearAllMarkers();
        return;
    }

    // ========================================================================================================
    // check collision with the interpolated path to create a real path
    // ========================================================================================================
    if (result.size() < 4)
    {
        RCLCPP_WARN(this->get_logger(), "Resulting path is too short for collision checking.");
        clearAllMarkers();
        return;
    }

    size_t index_to_collision = result.size();
    bool collision_detected_in_path = false; // Flag to track if collision was detected

    for (size_t i = 0; i + 3 < result.size(); i += 2)
    {
        double x1 = result[i];
        double y1 = result[i + 1];

        double x2 = result[i + 2];
        double y2 = result[i + 3];

        double yaw = atan2(y2 - y1, x2 - x1);

        State state;
        state.x = x1;
        state.y = y1;
        state.heading = yaw;

        bool collision = grid_map_->isSingleStateCollisionFreeImproved(state);
        if (collision)
        {
            index_to_collision = i;
            collision_detected_in_path = true; // Set flag when collision is detected
            break;
        }
    }

    // Prepare the Path message
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = rclcpp::Clock().now();

    // Extract the sampled points and add them to the path message
    for (size_t i = 0; i + 3 <= index_to_collision; i += 2)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = result[i];
        pose.pose.position.y = result[i + 1];
        pose.pose.position.z = 0.0;
        path_msg.poses.push_back(pose);
    }

    // Calculate the total path length
    double total_length_two = 0.0;
    for (size_t i = 1; i < path_msg.poses.size(); ++i)
    {
        const auto &prev_pose = path_msg.poses[i - 1].pose.position;
        const auto &curr_pose = path_msg.poses[i].pose.position;

        double dx = curr_pose.x - prev_pose.x;
        double dy = curr_pose.y - prev_pose.y;
        total_length_two += sqrt(dx * dx + dy * dy);
    }

    std::cout << green << "Total path length: " << total_length_two << " meters" << reset << std::endl;

    if (total_length_two < 1.0)
    {
        std::cout << red << "---> Total path length is less than 3m. Skipping path creation." << reset << std::endl;
        clearAllMarkers();
        return;
    }

    // Prepare the MarkerArray for visualization for the path
    visualization_msgs::msg::MarkerArray marker_array_next;
    std::set<int> current_marker_ids;
    int id = 4000;

    size_t total_states = result.size() / 2;

    for (size_t i = 0; i + 3 <= index_to_collision; i += 2)
    {
        if (i + 1 >= result.size())
            break;

        double x1 = result[i];
        double y1 = result[i + 1];

        double x2 = result[i + 2];
        double y2 = result[i + 3];

        double yaw = atan2(y2 - y1, x2 - x1);

        // Create the CUBE marker for the car polygon
        visualization_msgs::msg::Marker car_polygon_marker;
        car_polygon_marker.header.frame_id = "map";
        car_polygon_marker.header.stamp = this->now();
        car_polygon_marker.ns = "next_state_polygons";
        car_polygon_marker.action = visualization_msgs::msg::Marker::ADD;
        car_polygon_marker.id = id++;
        car_polygon_marker.type = visualization_msgs::msg::Marker::CUBE;

        current_marker_ids.insert(car_polygon_marker.id);

        // Set the size of the CUBE
        car_polygon_marker.scale.x = car_data_.axleToFront + car_data_.axleToBack;
        car_polygon_marker.scale.y = car_data_.width;
        car_polygon_marker.scale.z = 0.1;

        // NEW COLOR LOGIC: Red if collision detected, otherwise normal gradient
        double r = 0.0, g = 1.0, b = 0.0;
        
        if (collision_detected_in_path)
        {
            // Path is red when collision is detected
            r = 1.0;
            g = 0.0;
            b = 0.0;
        }
        else
        {
            // Normal gradient: Yellow -> Green -> Blue
            double t = (total_states > 1) ? static_cast<double>(i) / (total_states - 1) : 1.0;
            
            if (t < 0.5)
            {
                // Yellow to Green transition (first half)
                double yellow_to_green_factor = t / 0.5;
                r = 1.0 - yellow_to_green_factor;
                g = 1.0;
                b = 0.0;
            }
            else
            {
                // Green to Blue transition (second half)
                double green_to_blue_factor = (t - 0.5) / 0.5;
                r = 0.0;
                g = 1.0 - green_to_blue_factor;
                b = green_to_blue_factor;
            }
        }

        car_polygon_marker.color.r = r;
        car_polygon_marker.color.g = g;
        car_polygon_marker.color.b = b;
        car_polygon_marker.color.a = 0.9;

        // Set the pose of the CUBE
        car_polygon_marker.pose.position.x = result[i];
        car_polygon_marker.pose.position.y = result[i + 1];
        car_polygon_marker.pose.position.z = car_state_->z - 0.7;

        // Convert heading (yaw) to quaternion for marker orientation
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, yaw);
        car_polygon_marker.pose.orientation.x = quat.x();
        car_polygon_marker.pose.orientation.y = quat.y();
        car_polygon_marker.pose.orientation.z = quat.z();
        car_polygon_marker.pose.orientation.w = quat.w();

        marker_array_next.markers.push_back(car_polygon_marker);
    }

    // Identify and delete unused markers
    for (int previous_id : previous_marker_ids)
    {
        if (current_marker_ids.find(previous_id) == current_marker_ids.end())
        {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = this->now();
            delete_marker.ns = "next_state_polygons";
            delete_marker.id = previous_id;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;

            marker_array_next.markers.push_back(delete_marker);
        }
    }

    previous_marker_ids = current_marker_ids;

    // Publish the MarkerArray
    car_path_pub_->publish(marker_array_next);

    // Publish the interpolated path
    path_publisher_->publish(path_msg);

    // Log the collision status
    if (collision_detected_in_path)
    {
        std::cout << red << "---> Path published with RED color (collision detected)" << reset << std::endl;
    }
    else
    {
        std::cout << green << "---> Path published with normal gradient colors" << reset << std::endl;
    }
}

double local_path_planning_node::computeCost(const Eigen::VectorXd &point, const Eigen::VectorXd &prev_point, int layer_idx)
{
    // Obstacle cost calculation
    Eigen::Vector2d pos(point(0), point(1));
    double obstacle_cost = 0.0;
    double obstacle_distance = grid_map_->getObstacleDistance(pos);
    double safety_distance = 5.0;

    if (obstacle_distance < safety_distance)
    {
        obstacle_cost = (safety_distance - obstacle_distance) / safety_distance * FLAGS_search_obstacle_cost;
    }

    // check this improvement
    // if (obstacle_distance < safety_distance)
    // {
    //     if (obstacle_distance < 0.5)
    //     { // Too close, definite collision
    //         collision_cost = std::numeric_limits<double>::max();
    //     }
    //     else
    //     {
    //         collision_cost = (safety_distance - obstacle_distance) * FLAGS_search_obstacle_cost;
    //     }
    // }

    // Deviation cost (penalizes lateral deviation from the previous point)
    double offset_cost = fabs(point(1) - prev_point(1)) / FLAGS_search_lateral_range * FLAGS_search_deviation_cost;

    // Smoothness cost (penalizes sharp heading changes)
    double heading_diff = fabs(point(2) - prev_point(2));
    double smoothness_cost = heading_diff * FLAGS_smoothness_weight;

    // Curvature cost (penalizes sharp turns by considering the curvature between points)
    double curvature_cost = 0.0;
    if (layer_idx > 0) // Avoid computing curvature for the first layer
    {
        double dx = point(0) - prev_point(0);
        double dy = point(1) - prev_point(1);
        double distance = sqrt(dx * dx + dy * dy);

        if (distance > 0.1) // Prevent division by zero or near-zero
        {
            curvature_cost = fabs(heading_diff / distance) * FLAGS_curvature_weight;
        }
    }

    // Persistence penalty: Increase cost if changes are too small or short-lived
    static const int persistence_threshold = 4; // Minimum number of consecutive waypoints for a change
    double persistence_cost = 0.0;
    if (layer_idx < persistence_threshold)
    {
        persistence_cost = curvature_cost * FLAGS_persistence_weight;
    }

    // Proximity cost (encourages staying close to the reference path)
    double proximity_cost = 0.0;
    double min_distance_to_waypoint = std::numeric_limits<double>::max();

    // Define a search window around the current waypoint index
    int start_idx = std::max(0, layer_idx - 3);
    int end_idx = std::min(static_cast<int>(waypoints_segmentation->size()) - 1, layer_idx + 3);

    // Find the nearest waypoint within the limited range
    for (int i = start_idx; i <= end_idx; ++i)
    {
        const auto &waypoint = waypoints_segmentation->at(i);
        double distance = sqrt(pow(waypoint(0) - point(0), 2) + pow(waypoint(1) - point(1), 2));
        if (distance < min_distance_to_waypoint)
        {
            min_distance_to_waypoint = distance;
        }
    }

    // Apply proximity weight to encourage staying close to the reference path
    proximity_cost = min_distance_to_waypoint * FLAGS_proximity_weight;

    // Total cost
    return obstacle_cost + offset_cost * 0.5 + smoothness_cost * 0.3 + curvature_cost * 2.0 + persistence_cost + proximity_cost;
}

void local_path_planning_node::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    waypoints->clear();

    for (const auto &marker : msg->markers)
    {
        Eigen::VectorXd waypoint(4);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;

        tf2::Quaternion q(
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w);

        // Extract yaw from the quaternion
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        waypoint(3) = yaw;

        waypoints->push_back(waypoint);
    }
}

void local_path_planning_node::map_combination(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    if (!global_map_)
    {
        std::cout << red << "Warning: global_map is not available. Skipping map combination." << reset << std::endl;
        return;
    }
    // clean the rescaled_chunk_
    rescaled_chunk_->data.clear();

    // Compute grid_map_origin position
    State grid_map_origin;
    double forward_distance = 8.0; // Distance in meters

    // Calculate the position based on the car's heading
    grid_map_origin.x = car_state_->x + forward_distance * cos(car_state_->heading);
    grid_map_origin.y = car_state_->y + forward_distance * sin(car_state_->heading);
    grid_map_origin.z = car_state_->z; // Same height as car's position
    grid_map_origin.heading = car_state_->heading;

    // for the withe square
    State white_square;
    double forward_distance_square = 2.0; // Distance in meters

    // Calculate the position based on the car's heading
    white_square.x = car_state_->x + forward_distance_square * cos(car_state_->heading);
    white_square.y = car_state_->y + forward_distance_square * sin(car_state_->heading);
    white_square.z = car_state_->z; // Same height as car's position
    white_square.heading = car_state_->heading;

    int chunk_size = 50; // 20 x 20 meters
    int chunk_radius = chunk_size / 2;

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

    double scale_factor = 5;
    cv::Mat chunk_mat = toMat(chunk);
    cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);

    rescaled_chunk_->header = global_map_->header;
    rescaled_chunk_->info.resolution = 0.2;
    rescaled_chunk_->info.width = rescaled_chunk_mat.cols;
    rescaled_chunk_->info.height = rescaled_chunk_mat.rows;
    rescaled_chunk_->info.origin.position.x = chunk.info.origin.position.x;
    rescaled_chunk_->info.origin.position.y = chunk.info.origin.position.y;
    rescaled_chunk_->info.origin.position.z = car_state_->z;
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

    int inflation_radius = 0; // Inflated cells around the obstacles
    int value_to_mark = 100;

    // Transformation from lidar frame to map frame
    double cos_heading = cos(car_state_->heading);
    double sin_heading = sin(car_state_->heading);

    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        const auto &obstacle = msg->obstacles[i];
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

            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
    }

    if (mode_obstacle)
    {

        cout << yellow << "square white" << reset << endl;
        int square_size = 60; // Size of the square region in grid cells
        int half_square = square_size / 2;

        // Calculate the car's position in the rescaled grid
        double cos_heading = cos(white_square.heading);
        double sin_heading = sin(white_square.heading);

        int car_x_rescaled = static_cast<int>((white_square.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
        int car_y_rescaled = static_cast<int>((white_square.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

        // Iterate over the square and rotate points based on the heading
        for (int y = -half_square; y <= half_square; ++y)
        {
            for (int x = -half_square; x <= half_square; ++x)
            {
                // Rotate the point relative to the car's heading
                double rotated_x = x * cos_heading - y * sin_heading;
                double rotated_y = x * sin_heading + y * cos_heading;

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
    }

    // Publish the rescaled chunk
    if (occupancy_grid_pub_test_->get_subscription_count() > 0)
    {
        occupancy_grid_pub_test_->publish(*rescaled_chunk_);
    }
}

// ============================== Main Function for map rescale ==============================
cv::Mat local_path_planning_node::toMat(const nav_msgs::msg::OccupancyGrid &map)
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

cv::Mat local_path_planning_node::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

vector<pair<double, double>> local_path_planning_node::calculate_trajectory(double steering_angle, double wheelbase, int num_points)
{
    vector<pair<double, double>> path;

    if (fabs(steering_angle) < 1e-6)
    {
        double step_length = 0.5;
        for (int i = 0; i < num_points; ++i)
        {
            path.push_back(make_pair(start_offset + i * step_length, 0.0));
        }
    }
    else
    {
        double radius = wheelbase / tan(fabs(steering_angle));
        double angular_step = M_PI / 2 / num_points;

        double theta = 0; // Starting angle
        for (int i = 0; i <= num_points; ++i)
        {
            double x = radius * sin(theta) + start_offset; // Start start_offset m in front
            double y = radius * (1 - cos(theta));

            if (steering_angle < 0)
            {
                y = -y;
            }

            path.push_back(make_pair(x, y));
            theta += angular_step;
        }
    }

    return path;
}

void local_path_planning_node::extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length)
{
    if (path.empty())
        return;

    segment_x.push_back(path.front().first);
    segment_y.push_back(path.front().second);

    double accumulated_length = 0.0;
    double previous_x = path.front().first;
    double previous_y = path.front().second;

    for (size_t i = 1; i < path.size() && accumulated_length < length; ++i)
    {
        double current_x = path[i].first;
        double current_y = path[i].second;

        double dx = current_x - previous_x;
        double dy = current_y - previous_y;
        double segment_length = sqrt(dx * dx + dy * dy);

        accumulated_length += segment_length;

        if (accumulated_length >= length)
        {
            double excess_length = accumulated_length - length;
            double ratio = segment_length > 0 ? (segment_length - excess_length) / segment_length : 0;
            current_x = previous_x + ratio * dx;
            current_y = previous_y + ratio * dy;

            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            break;
        }
        else
        {
            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            previous_x = current_x;
            previous_y = current_y;
        }
    }
}

// callback functions
void local_path_planning_node::roadElementsCallback(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg)
{

    if (waypoints->empty())
    {
        std::cout << red << "Warning: waypoints is not available. Skipping crosswalk calculation." << reset << std::endl;
        return;
    }

    publish_crosswalk_polygon_markers(msg);

    waypoints_segmentation->clear();

    // calculate the percentage of the waypoints
    size_t total_waypoints = waypoints->size();
    percentage_completed = (static_cast<double>(closest_waypoint) / static_cast<double>(total_waypoints)) * 100.0;
    // Create a message object
    std_msgs::msg::Int32 percentage_msg;
    percentage_msg.data = static_cast<int>(percentage_completed); // Set the data field

    // Publish the message
    percentage_publisher_->publish(percentage_msg);

    traffic_information_msgs::msg::RoadElements first_crosswalk;
    bool found_first_crosswalk = false;

    // Pre-define a 1x1 meter square polygon at the origin
    geometry_msgs::msg::Polygon base_square;
    std::vector<std::pair<double, double>> square_offsets = {{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
    for (const auto &offset : square_offsets)
    {
        geometry_msgs::msg::Point32 p;
        p.x = offset.first;
        p.y = offset.second;
        base_square.points.push_back(p);
    }
    // Close the square by connecting back to the first point
    base_square.points.push_back(base_square.points[0]);

    for (size_t i = closest_waypoint; i < waypoints->size(); ++i)
    {
        const auto &waypoint = waypoints->at(i);
        double x = waypoint(0);
        double y = waypoint(1);

        // push bach of the waypoints are btw segment_start_index_temp and segment_start_index
        if (i >= closest_waypoint && i <= closest_waypoint + 40 && i < waypoints->size())
        {
            waypoints_segmentation->push_back(waypoint);
        }

    }

    // Add up to 5 extra waypoints after the segmentation
    for (size_t i = segment_start_index + 1; i <= segment_start_index + 7 && i < waypoints->size(); ++i)
    {
        waypoints_segmentation->push_back(waypoints->at(i));
    }

    // check if the crosswalk has been visited
    // crosswalk_visited_check(first_crosswalk);
    compute_path_polygon();

    // // Output the ID of the nearest crosswalk
    // cout << "First crosswalk id: " << first_crosswalk.id << endl;
    // cout << "Segment start index: " << segment_start_index_temp << endl;
    // cout << "Segment end index: " << segment_start_index << endl;

    // // cout in gren the waypoints size:
    cout << green << "waypoints size: " << waypoints->size() << reset << endl;
}

void local_path_planning_node::publish_crosswalk_polygon_markers(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto &element : msg->polygons)
    {
        if (element.type == "crosswalk" && std::find(skip_ids.begin(), skip_ids.end(), element.id) == skip_ids.end())
        {
            // Create a marker for this crosswalk polygon
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "crosswalk_polygons";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set scale for line thickness
            marker.scale.x = 0.1;  // Line width
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Set color (blue for crosswalks)
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            // Add all points from the crosswalk polygon
            for (const auto &point : element.points)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.0;
                marker.points.push_back(p);
            }

            // Close the polygon by adding the first point again
            if (!element.points.empty())
            {
                geometry_msgs::msg::Point first_point;
                first_point.x = element.points[0].x;
                first_point.y = element.points[0].y;
                first_point.z = 0.0;
                marker.points.push_back(first_point);
            }

            // Set orientation (identity quaternion)
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker_array.markers.push_back(marker);
        }
    }

    // Publish the marker array
    crosswalk_polygon_marker_publisher_->publish(marker_array);
}

void local_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{

    // Check if vehicle_path is available or valid
    if (vehicle_path->points.empty())
    {
        std::cout << red << "Warning: vehicle_path is not available. Skipping collision check." << reset << std::endl;
        return;
    }

    auto init_time = std::chrono::system_clock::now();

    getCurrentRobotState();

    collision_vector->clear();

    // geat type
    auto message = std_msgs::msg::Int32();
    message.data = 2;

    collision_detected_trajectory = false;
    collision_detected_path = false;
    for (const auto &obstacle : msg->obstacles)
    {
        bool current_collision = collision_checker.check_collision(*vehicle_path, obstacle.polygon);
        collision_vector->push_back(current_collision);
        if (current_collision)
        {
            collision_detected_trajectory = true;
            message.data = 0;
        }
    }

    // std::cout << green << "---> Collision detected trajectory: " << collision_detected_trajectory << reset << std::endl;

    gear_publisher_->publish(message);
    map_combination(msg);
    sampleLayersAlongPath();

    // bool msgs
    auto collision_message = std_msgs::msg::Bool();
    collision_message.data = collision_detected_trajectory;
    emergency_stop_publisher_->publish(collision_message);

    // if (collision_detected_trajectory)
    // {
    //     map_combination(msg);
    // }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time for collision checker and map conbination: " << duration << " ms" << reset << endl;
}

void local_path_planning_node::yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg)
{

    // auto init_time = std::chrono::system_clock::now();

    vehicle_path->points.clear();

    // Example: limiting yaw angle to 2 decimal places
    double rounded_yaw = std::round(msg->data * 100.0) / 100.0;

    // Calculate the trajectory of the car in base of the yaw angle
    vector<pair<double, double>> trajectory = calculate_trajectory(rounded_yaw, turning_radius, num_points);

    // extract the segment of the trajectory
    std::vector<double> segment_x, segment_y;
    extract_segment(trajectory, segment_x, segment_y, 3.5);

    // Prepare Marker for visualization
    visualization_msgs::msg::Marker lane_maker;
    lane_maker.header.frame_id = "map";
    lane_maker.header.stamp = this->now();

    lane_maker.ns = "vehicle_path";
    lane_maker.id = 0;
    lane_maker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_maker.action = visualization_msgs::msg::Marker::ADD;
    lane_maker.scale.x = 0.07;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5;
    lane_maker.color.a = 1.0;

    if (collision_detected_trajectory)
    {
        lane_maker.color.r = 1.0;
        lane_maker.color.g = 0.0;
        lane_maker.color.b = 0.0;
    }
    else
    {
        lane_maker.color.r = 0.0;
        lane_maker.color.g = 1.0;
        lane_maker.color.b = 0.0;
    }

    // Use car_state_ for transformations
    double car_x = car_state_->x;
    double car_y = car_state_->y;
    double car_heading = car_state_->heading;

    // Publish lane with the track of the car for the left side
    for (size_t i = 0; i < segment_x.size(); ++i)
    {
        double angle = atan2(segment_y[i] - segment_y[std::max(int(i) - 1, 0)], segment_x[i] - segment_x[std::max(int(i) - 1, 0)]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        // Calculate point in the car's local frame
        double local_x = segment_x[i] + offset_x;
        double local_y = segment_y[i] + offset_y;

        // Transform to map frame using car state
        double global_x = car_x + (local_x * cos(car_heading) - local_y * sin(car_heading));
        double global_y = car_y + (local_x * sin(car_heading) + local_y * cos(car_heading));

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = local_x;
        converted_point.y = local_y;
        converted_point.z = 0.0;
        vehicle_path->points.push_back(converted_point);

        geometry_msgs::msg::Point left_point;
        left_point.x = global_x;
        left_point.y = global_y;
        left_point.z = car_state_->z;
        lane_maker.points.push_back(left_point);
    }

    //  invertion of the right side to math with the left side
    for (size_t i = segment_x.size(); i-- > 0;)
    {
        int previous_index = (i > 0) ? (i - 1) : 0;
        double angle = atan2(segment_y[i] - segment_y[previous_index], segment_x[i] - segment_x[previous_index]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        // Calculate point in the car's local frame
        double local_x = segment_x[i] - offset_x;
        double local_y = segment_y[i] - offset_y;

        // Transform to map frame using car state
        double global_x = car_x + (local_x * cos(car_heading) - local_y * sin(car_heading));
        double global_y = car_y + (local_x * sin(car_heading) + local_y * cos(car_heading));

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = local_x;
        converted_point.y = local_y;
        converted_point.z = 0.0;
        vehicle_path->points.push_back(converted_point);

        geometry_msgs::msg::Point right_point;
        right_point.x = global_x;
        right_point.y = global_y;
        right_point.z = car_state_->z;
        lane_maker.points.push_back(right_point);
    }

    vehicle_path->points.push_back(vehicle_path->points.front());

    lane_maker.points.push_back(lane_maker.points.front());

    // auto end_time = std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    // cout << blue << "Execution time for path creation: " << duration << " ms" << reset << endl;

    if (lane_steering_publisher_->get_subscription_count() > 0)
    {
        lane_steering_publisher_->publish(lane_maker);
    }
}

void local_path_planning_node::globalMap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    global_map_ = map;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_path_planning_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}