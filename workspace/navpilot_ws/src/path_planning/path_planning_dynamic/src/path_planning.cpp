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

    this->get_parameter("maxSteerAngle", maxSteerAngle);
    this->get_parameter("wheelBase", wheelBase);
    this->get_parameter("axleToFront", axleToFront);
    this->get_parameter("axleToBack", axleToBack);
    this->get_parameter("width", width);
    this->get_parameter("pathLength", pathLength);
    this->get_parameter("step_car", step_car);
    this->get_parameter("tree_depth", tree_depth);
    this->get_parameter("branching_factor", branching_factor);

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
    
    real_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/real_nodes", 10);

    real_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_1", 10);

    real_trajectories_pub_2 = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_2", 10);

    // subcrition for the graph of the lane elements
    full_graph_publisher_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/full_graph", 10, std::bind(&path_planning::full_graph_callback, this, std::placeholders::_1));

    // -------------> Initialize the shared pointers  <------------
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    car_state_ = std::make_shared<State>();
    full_graph_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
    grid_map_ = nullptr;
    current_node = nullptr;

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

    // get the motion commans
    motionCommands();
    precomputeCommandSamples();
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

    // TreeFlat flat_a = generateTrajectoryTree_flat(current_node->Current_state);
    // int best_a = select_best_leaf(flat_a, current_node->Current_state);
    // // clearAllMarkers();
    // publishBestPathFromFlat(flat_a, best_a, 1); // green color for the flat implementation

    // TreeFlat flat;
    // int best = generateTrajectoryTree_AStar_flat(current_node->Current_state, flat);
    // publishBestPathFromFlat(flat, best, 1); // green color for the flat implementation


    TreeFlat flat_map;
    int best_map = generateTrajectoryTree_AStar_flat_map(current_node->Current_state, flat_map);
    publishBestPathFromFlat(flat_map, best_map, 2); // blue color for the A* implementation
    
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time for path selection: " << duration << " ms" << reset << endl;
}

// =============================
// callback for the graph of the lane elements
// =============================
void path_planning::full_graph_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    full_graph_ = msg;
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
// generate the trajectory tree in a flat structure
// =============================

TreeFlat
path_planning::generateTrajectoryTree_flat(const State& root_state)
{
    TreeFlat out;

    const int B = std::max(1, branching_factor);
    const int D = std::max(0, tree_depth);

    // Match legacy semantics: expand only (D - 1) levels
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    // Safety: if params changed but precompute wasn't called yet
    if ((int)precomputed_rel_.size() != B) {
        precomputeCommandSamples();
    }

    // Reserve a conservative upper bound
    size_t max_nodes = 1, powB = 1;
    for (int d = 0; d < EFFECTIVE_DEPTH; ++d) { powB *= (size_t)B; max_nodes += powB; }
    max_nodes = std::min(max_nodes, (size_t)200000);
    out.nodes.reserve(max_nodes);

    // Root
    FlatNode root;
    root.state  = root_state;
    root.parent = -1;
    root.cost   = 0.0;
    root.steer  = 0.0;
    root.dir    = 1;
    root.depth  = 0;
    out.nodes.push_back(root);

    std::vector<int> current_level;
    current_level.reserve((size_t)std::pow(B, EFFECTIVE_DEPTH));
    current_level.push_back(0);

    for (int depth = 0; depth < EFFECTIVE_DEPTH; ++depth)
    {
        std::vector<int> next_level;
        next_level.reserve(current_level.size() * (size_t)B);

        for (int idx : current_level)
        {
            const FlatNode& parent = out.nodes[idx];
            bool produced_child = false;

            // Precompute rotation from parent.heading once
            const double c = std::cos(parent.state.heading);
            const double s = std::sin(parent.state.heading);

            for (size_t ci = 0; ci < motionCommand.size(); ++ci)
            {
                const double steer = motionCommand[ci][0];
                const int    dir   = static_cast<int>(motionCommand[ci][1]);

                // Use the precomputed relative sequence for this command
                const auto& seq = precomputed_rel_[ci];

                bool rejected = false;
                State last = parent.state;

                // March through the precomputed local points and lift to map frame
                for (int k = 0; k < pathLength; ++k)
                {
                    const auto& r = seq[k];

                    State ns;
                    // Rigid transform: rotate by parent.heading and translate by parent (x,y)
                    ns.x = parent.state.x + c * r.x - s * r.y;
                    ns.y = parent.state.y + s * r.x + c * r.y;
                    ns.z = parent.state.z;                    // keep same z convention
                    ns.heading = parent.state.heading + r.heading;

                    // Grid mapping & collision (keep your current predicate semantics)
                    auto cell = grid_map_->toCellID(ns);
                    ns.gridx = std::get<0>(cell);
                    ns.gridy = std::get<1>(cell);

                    if (grid_map_->isSingleStateCollisionFreeImproved(ns)) {
                        rejected = true;
                        break;
                    }

                    last = ns;
                }

                if (rejected) continue;

                // Accept child
                FlatNode child;
                child.state  = last;
                child.parent = idx;
                child.cost   = parent.cost + pathLength * step_car;
                child.steer  = steer;
                child.dir    = dir;
                child.depth  = parent.depth + 1;

                out.nodes.push_back(child);
                next_level.push_back((int)out.nodes.size() - 1);
                produced_child = true;
            }

            if (!produced_child) {
                // No valid expansions → this is a leaf at this depth
                out.leaves.push_back(idx);
            }
        }

        std::cout << blue << "Level " << (depth + 1) << ": Generated "
                  << next_level.size() << " nodes" << reset << std::endl;

        current_level.swap(next_level);

        if (current_level.empty()) {
            std::cout << yellow << "No valid trajectories found at depth "
                      << (depth + 2) << ", stopping tree generation" << reset << std::endl;
            break;
        }
    }

    // Deepest reached are leaves
    out.leaves.insert(out.leaves.end(), current_level.begin(), current_level.end());

    std::cout << green << "Total real nodes (endpoints): " << out.leaves.size()
              << " out of " << out.nodes.size() << " total nodes" << reset << std::endl;

    return out;
}

// generateTrajectoryTree_flat map calculation

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

    // Logging similar to your BFS output
    std::vector<int> per_depth(EFFECTIVE_DEPTH + 1, 0);
    for (const auto& n : out.nodes)
        if (n.depth > 0 && n.depth <= EFFECTIVE_DEPTH) per_depth[n.depth]++;

    // for (int d = 1; d <= EFFECTIVE_DEPTH; ++d) {
    //     std::cout << blue << "Level " << d << ": Generated "
    //               << per_depth[d] << " nodes" << reset << std::endl;
    // }
    // std::cout << green << "Total real nodes (endpoints): "
    //           << out.leaves.size() << " out of " << out.nodes.size()
    //           << " total nodes" << reset << std::endl;

    return best_goal_idx;
}


inline void path_planning::build_chain_indices(
    const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const
{
    chain.clear();
    for (int i = leaf_idx; i != -1; i = flat.nodes[i].parent) chain.push_back(i);
    std::reverse(chain.begin(), chain.end()); // root -> leaf
}

double path_planning::score_leaf(const TreeFlat& flat, int leaf_idx, const State& start) const
{
    const auto& leaf = flat.nodes[leaf_idx].state;

    // Forward/lateral components in the start heading frame
    const double cs = std::cos(start.heading);
    const double ss = std::sin(start.heading);
    const double dx = leaf.x - start.x;
    const double dy = leaf.y - start.y;

    const double forward =  dx * cs + dy * ss;  // want BIG
    const double lateral = -dx * ss + dy * cs;  // want SMALL

    // Heading error vs start
    auto wrap = [](double a){ while (a >  M_PI) a -= 2*M_PI; while (a < -M_PI) a += 2*M_PI; return a; };
    const double head_err = std::fabs(wrap(leaf.heading - start.heading));

    // Sum |steer| along the chain (cheap in flat form)
    double steer_sum = 0.0;
    for (int i = leaf_idx; flat.nodes[i].parent != -1; i = flat.nodes[i].parent)
        steer_sum += std::fabs(flat.nodes[i].steer);

    // Minimize this cost
    const double cost =
        (-W_FORWARD * forward) +   // maximize forward progress
        ( W_LAT     * std::fabs(lateral)) +
        ( W_STEER   * steer_sum) +
        ( W_HEAD    * head_err);

    return cost;
}

int path_planning::select_best_leaf(const TreeFlat& flat, const State& start) const
{
    int best = -1;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int leaf_idx : flat.leaves) {
        const double c = score_leaf(flat, leaf_idx, start);
        if (c < best_cost) { best_cost = c; best = leaf_idx; }
    }
    return best;
}


std::shared_ptr<planner::Node>
path_planning::materialize_one_leaf_chain(const TreeFlat& flat, int leaf_idx)
{
    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    std::shared_ptr<planner::Node> parent_ptr; // null for root
    State seg_start = flat.nodes[ chain.front() ].state; // root state

    // Optional: map (steer,dir) to precomputed index
    auto find_cmd_index = [&](double steer, int dir)->int{
        for (size_t i = 0; i < motionCommand.size(); ++i)
            if (dir == (int)motionCommand[i][1] && std::abs(steer - motionCommand[i][0]) < 1e-9)
                return (int)i;
        return -1;
    };

    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[ chain[k] ];

        std::vector<State> seg;
        seg.reserve((size_t)pathLength);

        // Prefer precomputed relative sequence if available
        int ci = find_cmd_index(fn.steer, fn.dir);
        if (ci >= 0 && ci < (int)precomputed_rel_.size() &&
            (int)precomputed_rel_[ci].size() == pathLength)
        {
            const double c0 = std::cos(seg_start.heading);
            const double s0 = std::sin(seg_start.heading);
            for (int i = 0; i < pathLength; ++i) {
                const auto& r = precomputed_rel_[ci][i];
                State s;
                s.x = seg_start.x + c0 * r.x - s0 * r.y;
                s.y = seg_start.y + s0 * r.x + c0 * r.y;
                s.z = seg_start.z;
                s.heading = seg_start.heading + r.heading;
                auto cell = grid_map_->toCellID(s);
                s.gridx = std::get<0>(cell); s.gridy = std::get<1>(cell);
                seg.push_back(s);
            }
        } else {
            // Fallback: re-simulate with getVehicleStep (only one chain: fine)
            State s = seg_start;
            for (int i = 0; i < pathLength; ++i) {
                s = car_data_.getVehicleStep(s, fn.steer, fn.dir, step_car);
                auto cell = grid_map_->toCellID(s);
                s.gridx = std::get<0>(cell); s.gridy = std::get<1>(cell);
                seg.push_back(s);
            }
        }

        auto node_ptr = std::make_shared<planner::Node>(
            seg.back(), seg, fn.cost, fn.steer, fn.dir, std::weak_ptr<planner::Node>());

        if (parent_ptr) node_ptr->Parent = parent_ptr;
        parent_ptr = node_ptr;
        seg_start  = seg.back();
    }

    // Degenerate case: no movement
    if (!parent_ptr) {
        const auto& fn = flat.nodes[ leaf_idx ];
        std::vector<State> seg(1, fn.state);
        parent_ptr = std::make_shared<planner::Node>(
            fn.state, seg, fn.cost, fn.steer, fn.dir, std::weak_ptr<planner::Node>());
    }

    return parent_ptr;
}

// =============================
// generate the trajectory based on the A* algorithm
// =============================
int path_planning::generateTrajectoryTree_AStar_flat(const State& root_state, TreeFlat& out)
{
    out.nodes.clear();
    out.leaves.clear();

    const int B = std::max(1, branching_factor);
    const int D = std::max(0, tree_depth);
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    // Make sure samples exist for current commands
    if ((int)precomputed_rel_.size() != B) {
        precomputeCommandSamples();
    }

    // Precompute start-frame axes for forward/lateral projections
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
    root.cost   = 0.0;     // <-- g(root) = 0 (no terminal terms here)
    out.nodes.push_back(root);

    // Best goal found so far
    int    best_goal_idx   = -1;
    double best_goal_cost  = std::numeric_limits<double>::infinity();

    // OPEN and CLOSED
    std::priority_queue<PQItem> open;
    std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g; // lowest g seen for this state

    auto stateKey = [&](const State& s)->LatticeKey {
        return LatticeKey{ s.gridx, s.gridy, heading_bin(s.heading) };
    };

    // Heuristic lower bound from a node at depth d to any goal at EFFECTIVE_DEPTH
    auto h_lower_bound = [&](int depth)->double {
        const int remaining_segments = EFFECTIVE_DEPTH - depth;
        if (remaining_segments <= 0) return 0.0;
        const int remaining_steps = remaining_segments * pathLength;
        // Best case: straight line forward projection each step
        return -W_FORWARD * (remaining_steps * step_car);
    };

    // Push root
    {
        LatticeKey k = stateKey(root.state);
        best_g[k] = 0.0;
        const double f0 = 0.0 + h_lower_bound(0);
        open.push(PQItem{0, f0, 0.0});
    }

    // Fast lambda: expand parent->child for command index ci (returns child idx or -1 if rejected)
    auto expand_one = [&](int parent_idx, size_t ci)->int {
        const FlatNode& parent = out.nodes[parent_idx];

        // Rotate once for parent.heading
        const double cp = std::cos(parent.state.heading);
        const double sp = std::sin(parent.state.heading);

        const auto& seq = precomputed_rel_[ci];

        // Simulate & collision-check using precomputed local samples
        State last = parent.state;
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

            if (grid_map_->isSingleStateCollisionFreeImproved(ns)) {
                return -1; // reject whole segment
            }

            last = ns;
        }

        // Build child node
        FlatNode child;
        child.state  = last;
        child.parent = parent_idx;
        child.depth  = parent.depth + 1;
        child.steer  = motionCommand[ci][0];
        child.dir    = (int)motionCommand[ci][1];

        // Incremental g:
        //   steer penalty (per segment)
        const double steer_pen = W_STEER * std::fabs(child.steer);
        //   forward reward for this segment
        const double dx = (last.x - parent.state.x);
        const double dy = (last.y - parent.state.y);
        const double forward_inc =  dx * cs0 + dy * ss0;          // projection on start axis
        const double g_child = out.nodes[parent_idx].cost          // g(parent)
                             + steer_pen
                             - W_FORWARD * forward_inc;

        child.cost = g_child; // store g in cost

        // Duplicate suppression
        LatticeKey ck = stateKey(child.state);
        auto it = best_g.find(ck);
        if (it != best_g.end() && g_child >= it->second - 1e-12) {
            return -1; // dominated or equal
        }
        best_g[ck] = g_child;

        out.nodes.push_back(child);
        return (int)out.nodes.size() - 1;
    };

    // A* main
    while (!open.empty())
    {
        PQItem cur = open.top(); open.pop();

        const int idx   = cur.idx;
        const auto& fn  = out.nodes[idx];
        const double g  = fn.cost;
        const int    d  = fn.depth;

        // Stale queue entry?
        // (If this item was created with an older g, skip.)
        if (std::fabs(g - cur.g_copy) > 1e-12) {
            // We didn't store exact g in node when pushing? We did (child.cost = g_child).
            // If they differ, it's stale -> skip.
            continue;
        }

        // If this is a "goal" (max depth), compute true total cost (adds terminal terms)
        if (d == EFFECTIVE_DEPTH)
        {
            // Terminal contributions (relative to start)
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

            // A* termination: if nobody in OPEN can beat current best goal, stop
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
            // Otherwise keep going; a better goal may still be discovered.
            continue;
        }

        // Otherwise expand this node
        bool produced_child = false;
        for (size_t ci = 0; ci < motionCommand.size(); ++ci)
        {
            int child_idx = expand_one(idx, ci);
            if (child_idx < 0) continue;
            produced_child = true;

            const auto& ch = out.nodes[child_idx];
            const double h = h_lower_bound(ch.depth);
            const double f = ch.cost + h;
            open.push(PQItem{child_idx, f, ch.cost});
        }

        // If this node had no valid children, it’s a dead-end leaf: consider as goal too
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
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
        }
    }

    // Leaves (optional): we can mark the best only, or also collect deepest layer.
    // For compatibility with your publishers, put at least the best.
    out.leaves.clear();
    if (best_goal_idx >= 0) out.leaves.push_back(best_goal_idx);

    // Logging like before (approximate): count nodes per depth
    std::vector<int> per_depth(EFFECTIVE_DEPTH+1, 0);
    for (const auto& n : out.nodes) {
        if (n.depth>0 && n.depth<=EFFECTIVE_DEPTH) per_depth[n.depth]++;
    }
    for (int d=1; d<=EFFECTIVE_DEPTH; ++d) {
        std::cout << blue << "Level " << d << ": Generated "
                  << per_depth[d] << " nodes" << reset << std::endl;
    }
    std::cout << green << "Total real nodes (endpoints): "
              << out.leaves.size() << " out of " << out.nodes.size()
              << " total nodes" << reset << std::endl;

    return best_goal_idx;
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
// publish the trajectory
// =============================

void path_planning::clearAllMarkers()
{
    // Clear real trajectories
    visualization_msgs::msg::MarkerArray clear_array;
    visualization_msgs::msg::Marker clear_real;
    clear_real.header.frame_id = "map";
    clear_real.header.stamp = this->now();
    clear_real.ns = "real_trajectories";
    clear_real.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_array.markers.push_back(clear_real);
    
    clear_real.ns = "real_endpoints";
    clear_array.markers.push_back(clear_real);
    
    clear_real.ns = "real_trajectory_labels";
    clear_array.markers.push_back(clear_real);
    
    real_trajectories_pub_->publish(clear_array);
    
    std::cout << yellow << "Cleared all trajectory markers" << reset << std::endl;
}

// NEW: publish directly from Flat structure (no Node objects)
void path_planning::publishRealTrajectoriesFromFlat(const TreeFlat& flat)
{
    if (real_trajectories_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray msg;
    msg.markers.reserve(flat.leaves.size() * 3 + 1);

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.ns = "real_trajectories";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(clear);
    clear.ns = "real_endpoints";
    msg.markers.push_back(clear);
    clear.ns = "real_trajectory_labels";
    msg.markers.push_back(clear);

    // Helper to rebuild the chain of indices root->leaf
    auto build_chain = [&](int leaf_idx, std::vector<int>& chain) {
        chain.clear();
        for (int i = leaf_idx; i != -1; i = flat.nodes[i].parent) chain.push_back(i);
        std::reverse(chain.begin(), chain.end());
    };

    std::vector<int> chain;
    chain.reserve((size_t)tree_depth + 2);

    // Re-simulate each leaf path and emit markers
    for (size_t i = 0; i < flat.leaves.size(); ++i)
    {
        build_chain(flat.leaves[i], chain);

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "real_trajectories";
        line.id = static_cast<int>(i + 1);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.1;
        line.color.a = 1.0;

        // Color by depth
        int depth = static_cast<int>(chain.size()) - 1;
        if (depth == 1)      { line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; }
        else if (depth == 2) { line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; }
        else if (depth == 3) { line.color.r = 0.0; line.color.g = 0.5; line.color.b = 1.0; }
        else                 { line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0; }

        // Re-simulate segments along the chain using the same kinematics
        State s = flat.nodes[chain.front()].state; // root
        // Back up to the true root pose (your flat root is already at car pose)
        for (size_t k = 1; k < chain.size(); ++k)
        {
            const auto& fn = flat.nodes[chain[k]];
            for (int step = 0; step < pathLength; ++step)
            {
                s = car_data_.getVehicleStep(s, fn.steer, fn.dir, step_car);
                geometry_msgs::msg::Point p;
                p.x = s.x; p.y = s.y; p.z = s.z;
                line.points.push_back(std::move(p));
            }
        }
        if (!line.points.empty()) msg.markers.push_back(std::move(line));

        // Endpoint sphere
        const auto& end = flat.nodes[flat.leaves[i]].state;
        visualization_msgs::msg::Marker endpoint;
        endpoint.header.frame_id = "map";
        endpoint.header.stamp = this->now();
        endpoint.ns = "real_endpoints";
        endpoint.id = static_cast<int>(i);
        endpoint.type = visualization_msgs::msg::Marker::SPHERE;
        endpoint.action = visualization_msgs::msg::Marker::ADD;
        endpoint.pose.position.x = end.x;
        endpoint.pose.position.y = end.y;
        endpoint.pose.position.z = end.z + 0.2;
        endpoint.scale.x = 0.2; endpoint.scale.y = 0.2; endpoint.scale.z = 0.2;
        endpoint.color = msg.markers.back().color; // same as line
        endpoint.color.a = 0.8;
        msg.markers.push_back(std::move(endpoint));
    }

    real_trajectories_pub_->publish(msg);
}

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


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// to do: 
// make the inverese of inflate the obstacles. i have to fill them with black 
// make the integration with the path of the lanelet