#include "waypoints_routing.h"

waypoints_routing::waypoints_routing(/* args */) : Node("waypoints_routing_node")
{
    this->declare_parameter("map_path", "/home/genis/Downloads/Town10.osm");
    this->declare_parameter("start_lanelet_id", 0);
    this->declare_parameter("end_lanelet_id", 0);
    this->declare_parameter("show_full_graph", true);
    this->declare_parameter("waypoint_interval", 3.0);
    if (!readParameters())
        rclcpp::shutdown();

    x_offset_ = 0.0;
    y_offset_ = 25.0;

    waypoints_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_routing", 10);
    full_graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/full_graph", 10);
    neighbor_waypoints_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/neighbor_waypoints", 10);

    // Timer to publish periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&waypoints_routing::publishWaypoints, this));

    // Load the lanelet map
    lanelet::Origin origin({49, 8.4});
    lanelet::projection::LocalCartesianProjector projector(origin);
    lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);

    // for (auto &point : map->pointLayer)
    // {
    //     point.x() = point.attribute("local_x").asDouble().value();
    //     point.y() = point.attribute("local_y").asDouble().value();
    // }

    for (auto &point : map->pointLayer)
    {
        point.x() = point.attribute("local_x").asDouble().value() + x_offset_;
        point.y() = point.attribute("local_y").asDouble().value() + y_offset_;
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> waypoints_routing_node initialized.\033[0m");
    // cout the paraments in blue

    std::cout << blue << "start_lanelet_id: " << start_lanelet_id << reset << std::endl;
    std::cout << blue << "end_lanelet_id: " << end_lanelet_id << reset << std::endl;

    // Generate full graph visualization if enabled
    if (show_full_graph)
    {
        generateFullGraphVisualization(map);
    }
    
    // Then do the routing test
    lanelet_routing_test(map);
}

waypoints_routing::~waypoints_routing()
{
}

bool waypoints_routing::readParameters()
{
    if (!this->get_parameter("map_path", map_path_))
    {
        std::cout << "Failed to read parameter 'map_path' " << std::endl;
        return false;
    }

    if (!this->get_parameter("start_lanelet_id", start_lanelet_id))
    {
        std::cout << "Failed to read parameter 'start_lanelet_id' " << std::endl;
        return false;
    }

    if (!this->get_parameter("end_lanelet_id", end_lanelet_id))
    {
        std::cout << "Failed to read parameter 'end_lanelet_id' " << std::endl;
        return false;
    }

    if (!this->get_parameter("show_full_graph", show_full_graph))
    {
        std::cout << "Failed to read parameter 'show_full_graph' " << std::endl;
        return false;
    }

    if (!this->get_parameter("waypoint_interval", waypoint_interval))
    {
        std::cout << "Failed to read parameter 'waypoint_interval' " << std::endl;
        return false;
    }
    return true;
}

void waypoints_routing::lanelet_routing_test(lanelet::LaneletMapPtr &map)
{
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);

    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

    if (routingGraph)
    {
        std::cout << green << "Routing graph built successfully" << reset << std::endl;

        lanelet::ConstLanelet startLanelet = map->laneletLayer.get(start_lanelet_id);
        lanelet::ConstLanelet endLanelet = map->laneletLayer.get(end_lanelet_id);

        // Check if the goal lanelet is reachable from the start lanelet
        double maxRoutingCost = 500.0;
        auto reachableSet = routingGraph->reachableSet(startLanelet, maxRoutingCost);
        bool isReachable = std::find_if(reachableSet.begin(), reachableSet.end(),
                                        [&](const lanelet::ConstLanelet &ll)
                                        { return ll.id() == endLanelet.id(); }) != reachableSet.end();

        // cout isReachabl in blue
        if (!isReachable)
        {
            std::cout << red << "Goal lanelet is not reachable from the start lanelet." << reset << std::endl;
        }
        else
        {
            std::cout << green << "Goal lanelet is reachable from the start lanelet." << reset << std::endl;
            Optional<routing::Route> route = routingGraph->getRoute(startLanelet, endLanelet, 0);
            if (route)
            {
                std::cout << green << "Route found" << reset << std::endl;

                // provides the shortest path between the startLanelet and endLanelet within the route.
                routing::LaneletPath shortestPath = route->shortestPath();

                int waypoint_id = 0;
                for (const auto &lanelet : shortestPath)
                {
                    auto points = lanelet.centerline3d();
                    // auto points = lanelet.centerline2d();

                    for (size_t i = 0; i < points.size() - 1; ++i)
                    {

                        const auto &point = points[i];
                        const auto &next_point = points[i + 1];

                        // Calculate yaw based on difference between current and next point
                        double dx = next_point.x() - point.x();
                        double dy = next_point.y() - point.y();
                        double yaw = std::atan2(dy, dx);

                        visualization_msgs::msg::Marker waypoint_marker;
                        waypoint_marker.header.frame_id = "map";
                        waypoint_marker.header.stamp = this->now();
                        waypoint_marker.ns = "graph_waypoints";
                        waypoint_marker.id = waypoint_id++;
                        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
                        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

                        waypoint_marker.color.a = 1.0;

                        waypoint_marker.scale.x = 1.0;
                        waypoint_marker.scale.y = 0.5;
                        waypoint_marker.scale.z = 0.5;
                        waypoint_marker.color.r = 0.0;
                        waypoint_marker.color.g = 0.0;
                        waypoint_marker.color.b = 1.0;

                        // Set the position for the waypoint
                        waypoint_marker.pose.position.x = point.x();
                        waypoint_marker.pose.position.y = point.y();
                        waypoint_marker.pose.position.z = point.z();

                        // Set the orientation based on the calculated yaw
                        tf2::Quaternion quaternion;
                        quaternion.setRPY(0, 0, yaw); // Roll and pitch are 0 for a flat map
                        waypoint_marker.pose.orientation.x = quaternion.x();
                        waypoint_marker.pose.orientation.y = quaternion.y();
                        waypoint_marker.pose.orientation.z = quaternion.z();
                        waypoint_marker.pose.orientation.w = quaternion.w();

                        // Add the marker to the marker array
                        graph_waypoint_markers.markers.push_back(waypoint_marker);
                    }
                }
                waypoints_publisher_->publish(graph_waypoint_markers);

                std::cout << green << "Shortest path found" << reset << std::endl;

                // Generate waypoints for neighboring lanelets
                generateNeighborWaypoints(map, routingGraph, shortestPath);

                // graph_waypoint_markers.markers.clear();

                // returns the entire sequence of connected lanelets within the same lane as startLanelet.
                LaneletSequence fullLane = route->fullLane(startLanelet);
            }
        }
    }
}

void waypoints_routing::generateFullGraphVisualization(lanelet::LaneletMapPtr &map)
{
    std::cout << green << "Generating full graph visualization with " << waypoint_interval 
              << "m intervals..." << reset << std::endl;
    
    int waypoint_id = 0;
    int total_lanelets = 0;
    int road_lanelets = 0;
    int skipped_lanelets = 0;
    
    // Iterate through all lanelets in the map
    for (const auto &lanelet : map->laneletLayer)
    {
        total_lanelets++;
        
        // Skip crosswalks and other non-road lanelets
        if (lanelet.attribute("subtype").value() == "crosswalk" ||
            lanelet.attribute("subtype").value() == "walkway" ||
            lanelet.attribute("subtype").value() == "sidewalk" ||
            lanelet.attribute("subtype").value() == "footway")
        {
            skipped_lanelets++;
            continue;
        }
        
        // Only process lanelets that are roads
        if (lanelet.attribute("subtype").value() != "road" && 
            lanelet.attribute("subtype").value() != "highway" &&
            lanelet.attribute("subtype").value() != "primary" &&
            lanelet.attribute("subtype").value() != "secondary" &&
            lanelet.attribute("subtype").value() != "tertiary" &&
            lanelet.attribute("subtype").value() != "residential" &&
            lanelet.attribute("subtype").value() != "service" &&
            lanelet.attribute("subtype").value() != "unclassified")
        {
            // If no specific subtype is set, check if it has road-like attributes
            if (!lanelet.hasAttribute("subtype") || lanelet.attribute("subtype").value().empty())
            {
                // Check if it has speed limit or other road attributes
                if (!lanelet.hasAttribute("speed_limit") && 
                    !lanelet.hasAttribute("one_way") &&
                    !lanelet.hasAttribute("vehicle"))
                {
                    skipped_lanelets++;
                    continue; // Skip if it doesn't look like a road
                }
            }
            else
            {
                skipped_lanelets++;
                continue; // Skip other non-road subtypes
            }
        }
        
        road_lanelets++;
        
        auto points = lanelet.centerline3d();
        
        if (points.empty())
            continue;
            
        // Generate waypoints at regular intervals along the lanelet
        std::vector<lanelet::ConstPoint3d> waypoints;
        
        // Always add the first point
        waypoints.push_back(points[0]);
        
        // Calculate cumulative distance and add waypoints at intervals
        double cumulative_distance = 0.0;
        lanelet::ConstPoint3d last_waypoint = points[0];
        
        for (size_t i = 1; i < points.size(); ++i)
        {
            const auto &current_point = points[i];
            const auto &previous_point = points[i-1];
            
            // Calculate distance between consecutive points
            double dx = current_point.x() - previous_point.x();
            double dy = current_point.y() - previous_point.y();
            double dz = current_point.z() - previous_point.z();
            double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            cumulative_distance += segment_distance;
            
            // If we've accumulated enough distance, add a waypoint
            if (cumulative_distance >= waypoint_interval)
            {
                waypoints.push_back(current_point);
                cumulative_distance = 0.0;
                last_waypoint = current_point;
            }
        }
        
        // Always add the last point if it's not already added
        if (waypoints.empty() || waypoints.back().id() != points.back().id())
        {
            waypoints.push_back(points.back());
        }
        
        // Create markers for each waypoint
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto &point = waypoints[i];
            
            // Calculate direction for this waypoint
            double yaw = 0.0;
            if (i < waypoints.size() - 1)
            {
                const auto &next_point = waypoints[i + 1];
                double dx = next_point.x() - point.x();
                double dy = next_point.y() - point.y();
                yaw = std::atan2(dy, dx);
            }
            else if (i > 0)
            {
                // For the last point, use the direction from the previous point
                const auto &prev_point = waypoints[i - 1];
                double dx = point.x() - prev_point.x();
                double dy = point.y() - prev_point.y();
                yaw = std::atan2(dy, dx);
            }
            
            // Create waypoint marker
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "full_graph_waypoints";
            waypoint_marker.id = waypoint_id++;
            waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set color based on lanelet ID for variety
            waypoint_marker.color.a = 0.8;
            waypoint_marker.color.r = 0.2 + 0.6 * (lanelet.id() % 3) / 2.0;
            waypoint_marker.color.g = 0.2 + 0.6 * ((lanelet.id() + 1) % 3) / 2.0;
            waypoint_marker.color.b = 0.2 + 0.6 * ((lanelet.id() + 2) % 3) / 2.0;
            
            // Set scale for arrow
            waypoint_marker.scale.x = 2.0; // Arrow length
            waypoint_marker.scale.y = 0.3; // Arrow width
            waypoint_marker.scale.z = 0.3; // Arrow height
            
            // Set position
            waypoint_marker.pose.position.x = point.x();
            waypoint_marker.pose.position.y = point.y();
            waypoint_marker.pose.position.z = point.z();
            
            // Set orientation
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, yaw);
            waypoint_marker.pose.orientation.x = quaternion.x();
            waypoint_marker.pose.orientation.y = quaternion.y();
            waypoint_marker.pose.orientation.z = quaternion.z();
            waypoint_marker.pose.orientation.w = quaternion.w();
             
            // Add to marker array
            full_graph_markers.markers.push_back(waypoint_marker);
        }
    }
    
    // Publish the full graph
    full_graph_publisher_->publish(full_graph_markers);
    
    std::cout << green << "Full graph visualization generated with " << full_graph_markers.markers.size() 
              << " waypoints from " << road_lanelets << " road lanelets" << reset << std::endl;
    std::cout << blue << "Processed " << road_lanelets << " road lanelets, skipped " << skipped_lanelets 
              << " non-road lanelets out of " << total_lanelets << " total lanelets" << reset << std::endl;
}

void waypoints_routing::generateNeighborWaypoints(lanelet::LaneletMapPtr &map, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath)
{
    std::cout << green << "Generating neighbor waypoints for routing path..." << reset << std::endl;
    
    int waypoint_id = 0;
    std::set<lanelet::Id> processed_lanelets; // To avoid duplicates
    
    // Clear previous neighbor waypoints
    neighbor_waypoint_markers.markers.clear();
    
    // First, add waypoints from the main routing path
    for (const auto &path_lanelet : shortestPath)
    {
        if (processed_lanelets.find(path_lanelet.id()) != processed_lanelets.end())
            continue;
            
        processed_lanelets.insert(path_lanelet.id());
        
        // Generate waypoints for the main path lanelet
        auto points = path_lanelet.centerline3d();
        
        if (points.empty())
            continue;
            
        // Generate waypoints at regular intervals along the lanelet
        std::vector<lanelet::ConstPoint3d> waypoints;
        
        // Always add the first point
        waypoints.push_back(points[0]);
        
        // Calculate cumulative distance and add waypoints at intervals
        double cumulative_distance = 0.0;
        lanelet::ConstPoint3d last_waypoint = points[0];
        
        for (size_t i = 1; i < points.size(); ++i)
        {
            const auto &current_point = points[i];
            const auto &previous_point = points[i-1];
            
            // Calculate distance between consecutive points
            double dx = current_point.x() - previous_point.x();
            double dy = current_point.y() - previous_point.y();
            double dz = current_point.z() - previous_point.z();
            double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            cumulative_distance += segment_distance;
            
            // If we've accumulated enough distance, add a waypoint
            if (cumulative_distance >= waypoint_interval)
            {
                waypoints.push_back(current_point);
                cumulative_distance = 0.0;
                last_waypoint = current_point;
            }
        }
        
        // Always add the last point if it's not already added
        if (waypoints.empty() || waypoints.back().id() != points.back().id())
        {
            waypoints.push_back(points.back());
        }
        
        // Create markers for each waypoint in the main path
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            const auto &point = waypoints[i];
            
            // Calculate direction for this waypoint
            double yaw = 0.0;
            if (i < waypoints.size() - 1)
            {
                const auto &next_point = waypoints[i + 1];
                double dx = next_point.x() - point.x();
                double dy = next_point.y() - point.y();
                yaw = std::atan2(dy, dx);
            }
            else if (i > 0)
            {
                // For the last point, use the direction from the previous point
                const auto &prev_point = waypoints[i - 1];
                double dx = point.x() - prev_point.x();
                double dy = point.y() - prev_point.y();
                yaw = std::atan2(dy, dx);
            }
            
            // Create waypoint marker for main path
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "neighbor_waypoints";
            waypoint_marker.id = waypoint_id++;
            waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Main path - use blue color
            waypoint_marker.color.a = 0.8;
            waypoint_marker.color.r = 0.0;
            waypoint_marker.color.g = 0.0;
            waypoint_marker.color.b = 1.0;
            
            // Set scale for arrow
            waypoint_marker.scale.x = 1.5; // Arrow length
            waypoint_marker.scale.y = 0.2; // Arrow width
            waypoint_marker.scale.z = 0.2; // Arrow height
            
            // Set position
            waypoint_marker.pose.position.x = point.x();
            waypoint_marker.pose.position.y = point.y();
            waypoint_marker.pose.position.z = point.z();
            
            // Set orientation
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, yaw);
            waypoint_marker.pose.orientation.x = quaternion.x();
            waypoint_marker.pose.orientation.y = quaternion.y();
            waypoint_marker.pose.orientation.z = quaternion.z();
            waypoint_marker.pose.orientation.w = quaternion.w();
             
            // Add to marker array
            neighbor_waypoint_markers.markers.push_back(waypoint_marker);
        }
    }
    
    // Then, add waypoints from neighboring and reachable lanelets
    for (const auto &path_lanelet : shortestPath)
    {
        // Get all lanelets in the same lane (besides the current lanelet)
        ConstLanelets lane_lanelets = routingGraph->besides(path_lanelet);
        
        // Process each lanelet in the lane (excluding the main path lanelet)
        for (const auto &lanelet : lane_lanelets)
        {
            // Skip the main path lanelet and already processed lanelets
            if (lanelet.id() == path_lanelet.id() || processed_lanelets.find(lanelet.id()) != processed_lanelets.end())
                continue;
                
            processed_lanelets.insert(lanelet.id());
            
            auto points = lanelet.centerline3d();
            
            if (points.empty())
                continue;
                
            // Generate waypoints at regular intervals along the lanelet
            std::vector<lanelet::ConstPoint3d> waypoints;
            
            // Always add the first point
            waypoints.push_back(points[0]);
            
            // Calculate cumulative distance and add waypoints at intervals
            double cumulative_distance = 0.0;
            lanelet::ConstPoint3d last_waypoint = points[0];
            
            for (size_t i = 1; i < points.size(); ++i)
            {
                const auto &current_point = points[i];
                const auto &previous_point = points[i-1];
                
                // Calculate distance between consecutive points
                double dx = current_point.x() - previous_point.x();
                double dy = current_point.y() - previous_point.y();
                double dz = current_point.z() - previous_point.z();
                double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                cumulative_distance += segment_distance;
                
                // If we've accumulated enough distance, add a waypoint
                if (cumulative_distance >= waypoint_interval)
                {
                    waypoints.push_back(current_point);
                    cumulative_distance = 0.0;
                    last_waypoint = current_point;
                }
            }
            
            // Always add the last point if it's not already added
            if (waypoints.empty() || waypoints.back().id() != points.back().id())
            {
                waypoints.push_back(points.back());
            }
            
            // Create markers for each waypoint
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto &point = waypoints[i];
                
                // Calculate direction for this waypoint
                double yaw = 0.0;
                if (i < waypoints.size() - 1)
                {
                    const auto &next_point = waypoints[i + 1];
                    double dx = next_point.x() - point.x();
                    double dy = next_point.y() - point.y();
                    yaw = std::atan2(dy, dx);
                }
                else if (i > 0)
                {
                    // For the last point, use the direction from the previous point
                    const auto &prev_point = waypoints[i - 1];
                    double dx = point.x() - prev_point.x();
                    double dy = point.y() - prev_point.y();
                    yaw = std::atan2(dy, dx);
                }
                
                // Create waypoint marker
                visualization_msgs::msg::Marker waypoint_marker;
                waypoint_marker.header.frame_id = "map";
                waypoint_marker.header.stamp = this->now();
                waypoint_marker.ns = "neighbor_waypoints";
                waypoint_marker.id = waypoint_id++;
                waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
                waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Neighbor lanelets - use orange color
                waypoint_marker.color.a = 0.8;
                waypoint_marker.color.r = 1.0;
                waypoint_marker.color.g = 0.5;
                waypoint_marker.color.b = 0.0;
                
                // Set scale for arrow
                waypoint_marker.scale.x = 1.5; // Arrow length
                waypoint_marker.scale.y = 0.2; // Arrow width
                waypoint_marker.scale.z = 0.2; // Arrow height
                
                // Set position
                waypoint_marker.pose.position.x = point.x();
                waypoint_marker.pose.position.y = point.y();
                waypoint_marker.pose.position.z = point.z();
                
                // Set orientation
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw);
                waypoint_marker.pose.orientation.x = quaternion.x();
                waypoint_marker.pose.orientation.y = quaternion.y();
                waypoint_marker.pose.orientation.z = quaternion.z();
                waypoint_marker.pose.orientation.w = quaternion.w();
                 
                // Add to marker array
                neighbor_waypoint_markers.markers.push_back(waypoint_marker);
            }
        }
    }
    
    // Also include lanelets that branch off through curves/turns (following relationships)
    std::cout << blue << "Finding lanelets that branch off through curves..." << reset << std::endl;
    
    for (const auto &path_lanelet : shortestPath)
    {
        // Get lanelets that follow this path lanelet (branches, curves, etc.)
        auto following_lanelets = routingGraph->following(path_lanelet, true); // withLaneChanges = true
        
        std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has " << following_lanelets.size() << " following lanelets" << reset << std::endl;
        
        for (const auto &following_lanelet : following_lanelets)
        {
            // Skip if already processed
            if (processed_lanelets.find(following_lanelet.id()) != processed_lanelets.end())
                continue;
                
            // Skip if this is part of the main path
            bool isInMainPath = false;
            for (const auto &main_lanelet : shortestPath)
            {
                if (following_lanelet.id() == main_lanelet.id())
                {
                    isInMainPath = true;
                    break;
                }
            }
            if (isInMainPath)
                continue;
            
            // Check if the following lanelet goes in the same general direction
            if (!isSameDirection(path_lanelet, following_lanelet))
            {
                std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out due to direction" << reset << std::endl;
                continue;
            }
            
            // Check if this lanelet is not beyond the target
            if (isBeyondTarget(following_lanelet, shortestPath))
            {
                std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out due to being beyond target" << reset << std::endl;
                continue;
            }
            
            std::cout << green << "  Adding following lanelet " << following_lanelet.id() << " as purple arrow" << reset << std::endl;
                
            processed_lanelets.insert(following_lanelet.id());
            
            auto points = following_lanelet.centerline3d();
            
            if (points.empty())
                continue;
                
            // Generate waypoints at regular intervals along the lanelet
            std::vector<lanelet::ConstPoint3d> waypoints;
            
            // Always add the first point
            waypoints.push_back(points[0]);
            
            // Calculate cumulative distance and add waypoints at intervals
            double cumulative_distance = 0.0;
            lanelet::ConstPoint3d last_waypoint = points[0];
            
            for (size_t i = 1; i < points.size(); ++i)
            {
                const auto &current_point = points[i];
                const auto &previous_point = points[i-1];
                
                // Calculate distance between consecutive points
                double dx = current_point.x() - previous_point.x();
                double dy = current_point.y() - previous_point.y();
                double dz = current_point.z() - previous_point.z();
                double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                cumulative_distance += segment_distance;
                
                // If we've accumulated enough distance, add a waypoint
                if (cumulative_distance >= waypoint_interval)
                {
                    waypoints.push_back(current_point);
                    cumulative_distance = 0.0;
                    last_waypoint = current_point;
                }
            }
            
            // Always add the last point if it's not already added
            if (waypoints.empty() || waypoints.back().id() != points.back().id())
            {
                waypoints.push_back(points.back());
            }
            
            // Create markers for each waypoint
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto &point = waypoints[i];
                
                // Calculate direction for this waypoint
                double yaw = 0.0;
                if (i < waypoints.size() - 1)
                {
                    const auto &next_point = waypoints[i + 1];
                    double dx = next_point.x() - point.x();
                    double dy = next_point.y() - point.y();
                    yaw = std::atan2(dy, dx);
                }
                else if (i > 0)
                {
                    // For the last point, use the direction from the previous point
                    const auto &prev_point = waypoints[i - 1];
                    double dx = point.x() - prev_point.x();
                    double dy = point.y() - prev_point.y();
                    yaw = std::atan2(dy, dx);
                }
                
                // Create waypoint marker
                visualization_msgs::msg::Marker waypoint_marker;
                waypoint_marker.header.frame_id = "map";
                waypoint_marker.header.stamp = this->now();
                waypoint_marker.ns = "neighbor_waypoints";
                waypoint_marker.id = waypoint_id++;
                waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
                waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Branching lanelets - use purple color to distinguish from others
                waypoint_marker.color.a = 0.8;
                waypoint_marker.color.r = 0.8;
                waypoint_marker.color.g = 0.0;
                waypoint_marker.color.b = 0.8;
                
                // Set scale for arrow
                waypoint_marker.scale.x = 1.3; // Arrow length
                waypoint_marker.scale.y = 0.18; // Arrow width
                waypoint_marker.scale.z = 0.18; // Arrow height
                
                // Set position
                waypoint_marker.pose.position.x = point.x();
                waypoint_marker.pose.position.y = point.y();
                waypoint_marker.pose.position.z = point.z();
                
                // Set orientation
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw);
                waypoint_marker.pose.orientation.x = quaternion.x();
                waypoint_marker.pose.orientation.y = quaternion.y();
                waypoint_marker.pose.orientation.z = quaternion.z();
                waypoint_marker.pose.orientation.w = quaternion.w();
                 
                // Add to marker array
                neighbor_waypoint_markers.markers.push_back(waypoint_marker);
            }
        }
    }
    
    // Try alternative approach: find lanelets that are connected through adjacency
    std::cout << blue << "Finding lanelets connected through adjacency..." << reset << std::endl;
    
    for (const auto &path_lanelet : shortestPath)
    {
        // Try to find lanelets that are adjacent to this path lanelet
        auto left_lanelet = routingGraph->left(path_lanelet);
        auto right_lanelet = routingGraph->right(path_lanelet);
        auto adjacent_left = routingGraph->adjacentLeft(path_lanelet);
        auto adjacent_right = routingGraph->adjacentRight(path_lanelet);
        
        std::vector<lanelet::ConstLanelet> adjacent_lanelets;
        if (left_lanelet) adjacent_lanelets.push_back(*left_lanelet);
        if (right_lanelet) adjacent_lanelets.push_back(*right_lanelet);
        if (adjacent_left) adjacent_lanelets.push_back(*adjacent_left);
        if (adjacent_right) adjacent_lanelets.push_back(*adjacent_right);
        
        std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has " << adjacent_lanelets.size() << " adjacent lanelets" << reset << std::endl;
        
        for (const auto &adjacent_lanelet : adjacent_lanelets)
        {
            // Skip if already processed
            if (processed_lanelets.find(adjacent_lanelet.id()) != processed_lanelets.end())
                continue;
                
            // Skip if this is part of the main path
            bool isInMainPath = false;
            for (const auto &main_lanelet : shortestPath)
            {
                if (adjacent_lanelet.id() == main_lanelet.id())
                {
                    isInMainPath = true;
                    break;
                }
            }
            if (isInMainPath)
                continue;
            
            // Check if the adjacent lanelet goes in the same general direction
            if (!isSameDirection(path_lanelet, adjacent_lanelet))
            {
                std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id() << " filtered out due to direction" << reset << std::endl;
                continue;
            }
            
            // Check if this lanelet is not beyond the target
            if (isBeyondTarget(adjacent_lanelet, shortestPath))
            {
                std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id() << " filtered out due to being beyond target" << reset << std::endl;
                continue;
            }
            
            std::cout << green << "  Adding adjacent lanelet " << adjacent_lanelet.id() << " as purple arrow" << reset << std::endl;
                
            processed_lanelets.insert(adjacent_lanelet.id());
            
            auto points = adjacent_lanelet.centerline3d();
            
            if (points.empty())
                continue;
                
            // Generate waypoints at regular intervals along the lanelet
            std::vector<lanelet::ConstPoint3d> waypoints;
            
            // Always add the first point
            waypoints.push_back(points[0]);
            
            // Calculate cumulative distance and add waypoints at intervals
            double cumulative_distance = 0.0;
            lanelet::ConstPoint3d last_waypoint = points[0];
            
            for (size_t i = 1; i < points.size(); ++i)
            {
                const auto &current_point = points[i];
                const auto &previous_point = points[i-1];
                
                // Calculate distance between consecutive points
                double dx = current_point.x() - previous_point.x();
                double dy = current_point.y() - previous_point.y();
                double dz = current_point.z() - previous_point.z();
                double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                cumulative_distance += segment_distance;
                
                // If we've accumulated enough distance, add a waypoint
                if (cumulative_distance >= waypoint_interval)
                {
                    waypoints.push_back(current_point);
                    cumulative_distance = 0.0;
                    last_waypoint = current_point;
                }
            }
            
            // Always add the last point if it's not already added
            if (waypoints.empty() || waypoints.back().id() != points.back().id())
            {
                waypoints.push_back(points.back());
            }
            
            // Create markers for each waypoint
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto &point = waypoints[i];
                
                // Calculate direction for this waypoint
                double yaw = 0.0;
                if (i < waypoints.size() - 1)
                {
                    const auto &next_point = waypoints[i + 1];
                    double dx = next_point.x() - point.x();
                    double dy = next_point.y() - point.y();
                    yaw = std::atan2(dy, dx);
                }
                else if (i > 0)
                {
                    // For the last point, use the direction from the previous point
                    const auto &prev_point = waypoints[i - 1];
                    double dx = point.x() - prev_point.x();
                    double dy = point.y() - prev_point.y();
                    yaw = std::atan2(dy, dx);
                }
                
                // Create waypoint marker
                visualization_msgs::msg::Marker waypoint_marker;
                waypoint_marker.header.frame_id = "map";
                waypoint_marker.header.stamp = this->now();
                waypoint_marker.ns = "neighbor_waypoints";
                waypoint_marker.id = waypoint_id++;
                waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
                waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Adjacent lanelets - use purple color
                waypoint_marker.color.a = 0.8;
                waypoint_marker.color.r = 0.8;
                waypoint_marker.color.g = 0.0;
                waypoint_marker.color.b = 0.8;
                
                // Set scale for arrow
                waypoint_marker.scale.x = 1.3; // Arrow length
                waypoint_marker.scale.y = 0.18; // Arrow width
                waypoint_marker.scale.z = 0.18; // Arrow height
                
                // Set position
                waypoint_marker.pose.position.x = point.x();
                waypoint_marker.pose.position.y = point.y();
                waypoint_marker.pose.position.z = point.z();
                
                // Set orientation
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw);
                waypoint_marker.pose.orientation.x = quaternion.x();
                waypoint_marker.pose.orientation.y = quaternion.y();
                waypoint_marker.pose.orientation.z = quaternion.z();
                waypoint_marker.pose.orientation.w = quaternion.w();
                 
                // Add to marker array
                neighbor_waypoint_markers.markers.push_back(waypoint_marker);
            }
        }
    }
    
    // Additionally, find all reachable lanelets from the main path using routing graph
    std::cout << blue << "Finding additional reachable lanelets from routing graph..." << reset << std::endl;
    
    for (const auto &path_lanelet : shortestPath)
    {
        // Get all lanelets reachable from this path lanelet within a reasonable distance
        double maxRoutingCost = 100.0; // Adjust this value to control how far to search
        auto reachableSet = routingGraph->reachableSet(path_lanelet, maxRoutingCost);
        
        for (const auto &reachable_lanelet : reachableSet)
        {
            // Skip if already processed
            if (processed_lanelets.find(reachable_lanelet.id()) != processed_lanelets.end())
                continue;
                
            // Skip if this is part of the main path
            bool isInMainPath = false;
            for (const auto &main_lanelet : shortestPath)
            {
                if (reachable_lanelet.id() == main_lanelet.id())
                {
                    isInMainPath = true;
                    break;
                }
            }
            if (isInMainPath)
                continue;
            
            // Check if the reachable lanelet goes in the same general direction as the main path
            if (!isSameDirection(path_lanelet, reachable_lanelet))
                continue;
            
            // Check if this lanelet is not beyond the target (we don't want paths after the goal)
            if (isBeyondTarget(reachable_lanelet, shortestPath))
                continue;
                
            processed_lanelets.insert(reachable_lanelet.id());
            
            auto points = reachable_lanelet.centerline3d();
            
            if (points.empty())
                continue;
                
            // Generate waypoints at regular intervals along the lanelet
            std::vector<lanelet::ConstPoint3d> waypoints;
            
            // Always add the first point
            waypoints.push_back(points[0]);
            
            // Calculate cumulative distance and add waypoints at intervals
            double cumulative_distance = 0.0;
            lanelet::ConstPoint3d last_waypoint = points[0];
            
            for (size_t i = 1; i < points.size(); ++i)
            {
                const auto &current_point = points[i];
                const auto &previous_point = points[i-1];
                
                // Calculate distance between consecutive points
                double dx = current_point.x() - previous_point.x();
                double dy = current_point.y() - previous_point.y();
                double dz = current_point.z() - previous_point.z();
                double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                cumulative_distance += segment_distance;
                
                // If we've accumulated enough distance, add a waypoint
                if (cumulative_distance >= waypoint_interval)
                {
                    waypoints.push_back(current_point);
                    cumulative_distance = 0.0;
                    last_waypoint = current_point;
                }
            }
            
            // Always add the last point if it's not already added
            if (waypoints.empty() || waypoints.back().id() != points.back().id())
            {
                waypoints.push_back(points.back());
            }
            
            // Create markers for each waypoint
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                const auto &point = waypoints[i];
                
                // Calculate direction for this waypoint
                double yaw = 0.0;
                if (i < waypoints.size() - 1)
                {
                    const auto &next_point = waypoints[i + 1];
                    double dx = next_point.x() - point.x();
                    double dy = next_point.y() - point.y();
                    yaw = std::atan2(dy, dx);
                }
                else if (i > 0)
                {
                    // For the last point, use the direction from the previous point
                    const auto &prev_point = waypoints[i - 1];
                    double dx = point.x() - prev_point.x();
                    double dy = point.y() - prev_point.y();
                    yaw = std::atan2(dy, dx);
                }
                
                // Create waypoint marker
                visualization_msgs::msg::Marker waypoint_marker;
                waypoint_marker.header.frame_id = "map";
                waypoint_marker.header.stamp = this->now();
                waypoint_marker.ns = "neighbor_waypoints";
                waypoint_marker.id = waypoint_id++;
                waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
                waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Reachable lanelets - use green color to distinguish from direct neighbors
                waypoint_marker.color.a = 0.7;
                waypoint_marker.color.r = 0.0;
                waypoint_marker.color.g = 1.0;
                waypoint_marker.color.b = 0.0;
                
                // Set scale for arrow
                waypoint_marker.scale.x = 1.2; // Slightly smaller than main path
                waypoint_marker.scale.y = 0.15; // Arrow width
                waypoint_marker.scale.z = 0.15; // Arrow height
                
                // Set position
                waypoint_marker.pose.position.x = point.x();
                waypoint_marker.pose.position.y = point.y();
                waypoint_marker.pose.position.z = point.z();
                
                // Set orientation
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw);
                waypoint_marker.pose.orientation.x = quaternion.x();
                waypoint_marker.pose.orientation.y = quaternion.y();
                waypoint_marker.pose.orientation.z = quaternion.z();
                waypoint_marker.pose.orientation.w = quaternion.w();
                 
                // Add to marker array
                neighbor_waypoint_markers.markers.push_back(waypoint_marker);
            }
        }
    }
    
    // Publish the neighbor waypoints
    neighbor_waypoints_publisher_->publish(neighbor_waypoint_markers);
    
    std::cout << green << "Neighbor waypoints generated with " << neighbor_waypoint_markers.markers.size() 
              << " waypoints from " << processed_lanelets.size() << " lanelets (including main path, direct neighbors, branching curves, and reachable lanelets)" << reset << std::endl;
}

bool waypoints_routing::isSameDirection(const lanelet::ConstLanelet &lanelet1, const lanelet::ConstLanelet &lanelet2)
{
    // Get the centerlines of both lanelets
    auto points1 = lanelet1.centerline3d();
    auto points2 = lanelet2.centerline3d();
    
    if (points1.size() < 2 || points2.size() < 2)
        return false;
    
    // Calculate the direction vector for lanelet1 (from start to end)
    double dx1 = points1.back().x() - points1.front().x();
    double dy1 = points1.back().y() - points1.front().y();
    double length1 = std::sqrt(dx1*dx1 + dy1*dy1);
    
    if (length1 < 1e-6) // Avoid division by zero
        return false;
    
    // Normalize the direction vector for lanelet1
    dx1 /= length1;
    dy1 /= length1;
    
    // Calculate the direction vector for lanelet2 (from start to end)
    double dx2 = points2.back().x() - points2.front().x();
    double dy2 = points2.back().y() - points2.front().y();
    double length2 = std::sqrt(dx2*dx2 + dy2*dy2);
    
    if (length2 < 1e-6) // Avoid division by zero
        return false;
    
    // Normalize the direction vector for lanelet2
    dx2 /= length2;
    dy2 /= length2;
    
    // Calculate the dot product to determine if they're going in the same direction
    double dot_product = dx1 * dx2 + dy1 * dy2;
    
    // Use a more relaxed threshold to include more lanelets
    // Allow lanelets that are going in roughly the same direction
    double threshold = 0.3; // cos(70°) ≈ 0.34, so this allows up to ~70° difference
    
    // Debug output
    std::cout << yellow << "    Direction check: lanelet1=" << lanelet1.id() 
              << " lanelet2=" << lanelet2.id() << " dot_product=" << dot_product 
              << " threshold=" << threshold << " result=" << (dot_product > threshold) << reset << std::endl;
    
    return dot_product > threshold;
}

bool waypoints_routing::isBeyondTarget(const lanelet::ConstLanelet &lanelet, const routing::LaneletPath &shortestPath)
{
    if (shortestPath.empty())
        return false;
    
    // Get the target lanelet (last lanelet in the path)
    const auto &target_lanelet = shortestPath.back();
    
    // Get centerlines
    auto lanelet_points = lanelet.centerline3d();
    auto target_points = target_lanelet.centerline3d();
    
    if (lanelet_points.empty() || target_points.empty())
        return false;
    
    // Get the end point of the target lanelet
    auto target_end = target_points.back();
    
    // Get the start point of the candidate lanelet
    auto lanelet_start = lanelet_points.front();
    
    // Calculate the direction from target end to lanelet start
    double dx = lanelet_start.x() - target_end.x();
    double dy = lanelet_start.y() - target_end.y();
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // Calculate the direction vector of the target lanelet
    double target_dx = target_end.x() - target_points.front().x();
    double target_dy = target_end.y() - target_points.front().y();
    double target_length = std::sqrt(target_dx*target_dx + target_dy*target_dy);
    
    if (target_length > 1e-6)
    {
        target_dx /= target_length;
        target_dy /= target_length;
        
        // Calculate dot product to see if lanelet is in the same direction as target
        double dot_product = (dx / distance) * target_dx + (dy / distance) * target_dy;
        
        // If dot product is positive, the lanelet is in the same direction as target (beyond target)
        // If dot product is negative, the lanelet is in opposite direction (before target)
        if (dot_product > 0.3) // Same direction as target
        {
            // Check if it's close enough to be a continuation
            if (distance < 15.0)
            {
                // This is likely a continuation beyond the target
                return true;
            }
        }
        else if (dot_product < -0.3) // Opposite direction from target
        {
            // This is before the target, should be included
            return false;
        }
    }
    
    // For lanelets that are perpendicular or unclear, be conservative
    // Only filter out if they're very close to target end (likely continuations)
    return distance < 5.0;
}

void waypoints_routing::publishWaypoints()
{
    waypoints_publisher_->publish(graph_waypoint_markers);
    if (show_full_graph)
    {
        full_graph_publisher_->publish(full_graph_markers);
    }
    neighbor_waypoints_publisher_->publish(neighbor_waypoint_markers);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<waypoints_routing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}