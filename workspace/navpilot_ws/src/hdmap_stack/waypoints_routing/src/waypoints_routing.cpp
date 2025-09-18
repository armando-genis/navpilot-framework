#include "waypoints_routing.h"
#include <limits>

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

void waypoints_routing::generateNeighborWaypoints(lanelet::LaneletMapPtr &/*map*/, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath)
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
            
            std::cout << blue << "  Evaluating following lanelet " << following_lanelet.id() << " for trajectory compatibility..." << reset << std::endl;
            
            //  More strict direction and trajectory checking
            if (!isCompatibleTrajectory(path_lanelet, following_lanelet, routingGraph, shortestPath))
            {
                std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out due to incompatible trajectory" << reset << std::endl;
                continue;
            }
            
            // Check if this is actually a branching lanelet (not just a parallel neighbor)
            if (!isBranchingLanelet(path_lanelet, following_lanelet))
            {
                std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out - not a branching lanelet" << reset << std::endl;
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
            
            std::cout << blue << "  Evaluating adjacent lanelet " << adjacent_lanelet.id() << " for trajectory compatibility..." << reset << std::endl;
            
            // IMPROVED: Use the same strict trajectory checking for adjacent lanelets
            if (!isCompatibleTrajectory(path_lanelet, adjacent_lanelet, routingGraph, shortestPath))
            {
                std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id() << " filtered out due to incompatible trajectory" << reset << std::endl;
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
    
    // Publish the neighbor waypoints
    neighbor_waypoints_publisher_->publish(neighbor_waypoint_markers);
    
    std::cout << green << "Neighbor waypoints generated with " << neighbor_waypoint_markers.markers.size() 
              << " waypoints from " << processed_lanelets.size() << " lanelets (including main path, direct neighbors, branching curves, and reachable lanelets)" << reset << std::endl;
}

// NEW FUNCTION: Curve-aware trajectory compatibility checking
bool waypoints_routing::isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath)
{
    auto path_points = path_lanelet.centerline3d();
    auto candidate_points = candidate_lanelet.centerline3d();
    
    if (path_points.size() < 3 || candidate_points.size() < 3)
        return false;
    
    // 1. Calculate connection quality - how well do the paths connect?
    double connection_distance = std::sqrt(
        std::pow(candidate_points.front().x() - path_points.back().x(), 2) +
        std::pow(candidate_points.front().y() - path_points.back().y(), 2)
    );
    
    bool is_direct_continuation = connection_distance < 10.0; // Within 10m = likely continuation
    
    std::cout << yellow << "    Connection analysis: distance=" << connection_distance 
              << " is_continuation=" << (is_direct_continuation ? "YES" : "NO") << reset << std::endl;
    
    // 2. Check overall direction compatibility
    double path_dx = path_points.back().x() - path_points.front().x();
    double path_dy = path_points.back().y() - path_points.front().y();
    double path_length = std::sqrt(path_dx*path_dx + path_dy*path_dy);
    
    double candidate_dx = candidate_points.back().x() - candidate_points.front().x();
    double candidate_dy = candidate_points.back().y() - candidate_points.front().y();
    double candidate_length = std::sqrt(candidate_dx*candidate_dx + candidate_dy*candidate_dy);
    
    if (path_length < 1e-6 || candidate_length < 1e-6)
        return false;
    
    // Normalize direction vectors
    path_dx /= path_length;
    path_dy /= path_length;
    candidate_dx /= candidate_length;
    candidate_dy /= candidate_length;
    
    double overall_dot = path_dx * candidate_dx + path_dy * candidate_dy;
    double overall_angle = std::acos(std::max(-1.0, std::min(1.0, overall_dot))) * 180.0 / M_PI;
    
    // Adaptive thresholds based on context
    double max_allowed_angle = is_direct_continuation ? 70.0 : 45.0; // Even more lenient for continuations
    
    std::cout << yellow << "    Overall direction: angle=" << overall_angle 
              << "° max_allowed=" << max_allowed_angle << "°" << reset << std::endl;
    
    if (overall_angle > max_allowed_angle)
    {
        std::cout << yellow << "    Trajectory rejected: poor overall direction alignment" << reset << std::endl;
        return false;
    }
    
    // 3. Cross product analysis for turn detection
    double cross_product = path_dx * candidate_dy - path_dy * candidate_dx;
    double cross_magnitude = std::abs(cross_product);
    
    std::cout << yellow << "    Cross product: magnitude=" << cross_magnitude 
              << " direction=" << (cross_product > 0 ? "left" : "right") << reset << std::endl;
    
    // 4. NEW: Check if this is a consistent diverging curve by analyzing the path end direction
    // Get the direction at the end of the path lanelet
    std::vector<lanelet::ConstPoint3d> path_vector(path_points.begin(), path_points.end());
    std::vector<lanelet::ConstPoint3d> candidate_vector(candidate_points.begin(), candidate_points.end());
    
    auto path_end_direction = getEndDirection(path_vector);
    auto candidate_start_direction = getStartDirection(candidate_vector);
    
    double transition_dot = path_end_direction.first * candidate_start_direction.first + 
                           path_end_direction.second * candidate_start_direction.second;
    double transition_angle = std::acos(std::max(-1.0, std::min(1.0, transition_dot))) * 180.0 / M_PI;
    
    std::cout << yellow << "    Transition angle: " << transition_angle << "°" << reset << std::endl;
    
    // Check for sharp direction changes at the connection point
    if (transition_angle > 90.0) // Very sharp transition
    {
        std::cout << yellow << "    Trajectory rejected: sharp transition at connection" << reset << std::endl;
        return false;
    }
    
    // 5. Progressive trajectory analysis - CURVE AWARE
    int num_samples = std::min(5, (int)std::min(path_points.size(), candidate_points.size()) - 1);
    double cumulative_deviation = 0.0;
    int valid_samples = 0;
    
    // Track if trajectory is consistently turning in one direction
    double cumulative_cross = 0.0;
    int cross_sign_changes = 0;
    double prev_cross = 0.0;
    
    for (int i = 0; i < num_samples; ++i)
    {
        int path_idx = (i * (path_points.size() - 1)) / num_samples;
        int candidate_idx = (i * (candidate_points.size() - 1)) / num_samples;
        
        if (path_idx + 1 >= path_points.size() || candidate_idx + 1 >= candidate_points.size())
            continue;
        
        // Calculate segment directions
        double path_seg_dx = path_points[path_idx + 1].x() - path_points[path_idx].x();
        double path_seg_dy = path_points[path_idx + 1].y() - path_points[path_idx].y();
        double path_seg_len = std::sqrt(path_seg_dx*path_seg_dx + path_seg_dy*path_seg_dy);
        
        double cand_seg_dx = candidate_points[candidate_idx + 1].x() - candidate_points[candidate_idx].x();
        double cand_seg_dy = candidate_points[candidate_idx + 1].y() - candidate_points[candidate_idx].y();
        double cand_seg_len = std::sqrt(cand_seg_dx*cand_seg_dx + cand_seg_dy*cand_seg_dy);
        
        if (path_seg_len < 1e-6 || cand_seg_len < 1e-6)
            continue;
        
        // Normalize
        path_seg_dx /= path_seg_len;
        path_seg_dy /= path_seg_len;
        cand_seg_dx /= cand_seg_len;
        cand_seg_dy /= cand_seg_len;
        
        // Calculate deviation
        double segment_dot = path_seg_dx * cand_seg_dx + path_seg_dy * cand_seg_dy;
        double deviation = std::acos(std::max(-1.0, std::min(1.0, segment_dot)));
        
        cumulative_deviation += deviation;
        
        // Track turning consistency
        double segment_cross = path_seg_dx * cand_seg_dy - path_seg_dy * cand_seg_dx;
        cumulative_cross += segment_cross;
        
        // Count sign changes in cross product (indicates inconsistent turning)
        if (valid_samples > 0 && ((prev_cross > 0) != (segment_cross > 0)) && std::abs(segment_cross) > 0.1)
        {
            cross_sign_changes++;
        }
        prev_cross = segment_cross;
        
        valid_samples++;
    }
    
    if (valid_samples == 0)
        return false;
    
    double avg_deviation = cumulative_deviation / valid_samples;
    double avg_deviation_deg = avg_deviation * 180.0 / M_PI;
    
    // MORE LENIENT: Allow curves to have higher average deviation
    double max_avg_deviation = is_direct_continuation ? 50.0 : 35.0; // Increased limits
    
    std::cout << yellow << "    Segment analysis: avg_dev=" << avg_deviation_deg 
              << "° cross_sign_changes=" << cross_sign_changes << reset << std::endl;
    
    if (avg_deviation_deg > max_avg_deviation)
    {
        std::cout << yellow << "    Trajectory rejected: excessive average deviation" << reset << std::endl;
        return false;
    }
    
    // 6. CONNECTIVITY-BASED: Check if candidate connects to valid lanelets
    double avg_cross = cumulative_cross / valid_samples;
    
    std::cout << yellow << "    Turn analysis: avg_cross=" << avg_cross << reset << std::endl;
    
    // IMPROVED APPROACH: Check if candidate leads to useful destinations, not just connections
    
    // Get the lanelets that follow the candidate
    auto candidate_following = routingGraph->following(candidate_lanelet, true);
    auto candidate_previous = routingGraph->previous(candidate_lanelet, true);
    
    // Find the current path lanelet index
    int current_path_index = -1;
    for (size_t i = 0; i < shortestPath.size(); ++i)
    {
        if (shortestPath[i].id() == path_lanelet.id())
        {
            current_path_index = i;
            break;
        }
    }
    
    // Use helper function to count meaningful connections
    int meaningful_connections = countMeaningfulConnections(candidate_lanelet, routingGraph, shortestPath, current_path_index);
    
    std::cout << yellow << "    Connectivity analysis: meaningful_connections=" << meaningful_connections << reset << std::endl;
    
    // Require at least 2 meaningful connections AND both start/end connections for valid curves/paths
    // This ensures lanelets connect to multiple blue/orange paths and aren't dead-ends
    if (meaningful_connections < 2)
    {
        std::cout << yellow << "    Trajectory rejected: insufficient meaningful connections (" << meaningful_connections 
                  << " < 2). Valid curves/paths must connect to at least 2 different lanelets or destinations." << reset << std::endl;
        return false;
    }
    
    // Additional validation: Check if meaningful_connections includes both start and end connections
    // This is handled by countMeaningfulConnections() which now requires both start_connections > 0 AND end_connections > 0
    
    // Additional check: If it connects but has very strong divergent turning, still be suspicious
    if (std::abs(avg_cross) > 0.8 && meaningful_connections < 2)
    {
        // Very strong turning + very limited connections = suspicious
        std::cout << yellow << "    Trajectory rejected: very strong divergent turning with very limited connections" << reset << std::endl;
        return false;
    }
    
    // Keep a safety threshold for extremely sharp turns
    if (std::abs(avg_cross) > 1.2)
    {
        std::cout << yellow << "    Trajectory rejected: extremely sharp turning" << reset << std::endl;
        return false;
    }
    
    // 7. Distance progression check - MORE LENIENT
    double start_distance = std::sqrt(
        std::pow(candidate_points.front().x() - path_points.front().x(), 2) +
        std::pow(candidate_points.front().y() - path_points.front().y(), 2)
    );
    
    double end_distance = std::sqrt(
        std::pow(candidate_points.back().x() - path_points.back().x(), 2) +
        std::pow(candidate_points.back().y() - path_points.back().y(), 2)
    );
    
    std::cout << yellow << "    Distance check: start=" << start_distance << " end=" << end_distance << reset << std::endl;
    
    // Much more lenient for direct continuations, especially curves
    double max_divergence_factor = is_direct_continuation ? 1.0 : 0.4; // Very lenient for continuations
    double max_lanelet_length = std::max(path_length, candidate_length);
    
    if (end_distance > start_distance + max_lanelet_length * max_divergence_factor)
    {
        std::cout << yellow << "    Trajectory rejected: excessive divergence" << reset << std::endl;
        return false;
    }
    
    std::cout << green << "    Trajectory accepted: compatible path (continuation=" 
              << (is_direct_continuation ? "YES" : "NO") << ")" << reset << std::endl;
    return true;
}

// Helper function to get direction at end of path
std::pair<double, double> waypoints_routing::getEndDirection(const std::vector<lanelet::ConstPoint3d> &points)
{
    if (points.size() < 2)
        return {0, 0};
    
    // Use last few points for better direction estimate
    int start_idx = std::max(0, (int)points.size() - 3);
    int end_idx = points.size() - 1;
    
    double dx = points[end_idx].x() - points[start_idx].x();
    double dy = points[end_idx].y() - points[start_idx].y();
    double length = std::sqrt(dx*dx + dy*dy);
    
    if (length < 1e-6)
        return {0, 0};
    
    return {dx/length, dy/length};
}

// Helper function to get direction at start of path
std::pair<double, double> waypoints_routing::getStartDirection(const std::vector<lanelet::ConstPoint3d> &points)
{
    if (points.size() < 2)
        return {0, 0};
    
    // Use first few points for better direction estimate
    int start_idx = 0;
    int end_idx = std::min(2, (int)points.size() - 1);
    
    double dx = points[end_idx].x() - points[start_idx].x();
    double dy = points[end_idx].y() - points[start_idx].y();
    double length = std::sqrt(dx*dx + dy*dy);
    
    if (length < 1e-6)
        return {0, 0};
    
    return {dx/length, dy/length};
}

// Helper function to calculate path curvature
double waypoints_routing::calculatePathCurvature(const std::vector<lanelet::ConstPoint3d> &points)
{
    if (points.size() < 3)
        return 0.0;
    
    // Sample 3-5 points along the path to calculate average curvature
    int num_samples = std::min(5, (int)points.size() - 2);
    double cumulative_curvature = 0.0;
    int valid_samples = 0;
    
    for (int i = 0; i < num_samples; ++i)
    {
        int idx = (i * (points.size() - 2)) / num_samples + 1; // Start from index 1
        
        if (idx - 1 < 0 || idx + 1 >= points.size())
            continue;
        
        // Get three consecutive points
        auto p1 = points[idx - 1];
        auto p2 = points[idx];
        auto p3 = points[idx + 1];
        
        // Calculate vectors
        double v1x = p2.x() - p1.x();
        double v1y = p2.y() - p1.y();
        double v2x = p3.x() - p2.x();
        double v2y = p3.y() - p2.y();
        
        double len1 = std::sqrt(v1x*v1x + v1y*v1y);
        double len2 = std::sqrt(v2x*v2x + v2y*v2y);
        
        if (len1 < 1e-6 || len2 < 1e-6)
            continue;
        
        // Normalize vectors
        v1x /= len1;
        v1y /= len1;
        v2x /= len2;
        v2y /= len2;
        
        // Calculate curvature using cross product (positive = left turn, negative = right turn)
        double curvature = v1x * v2y - v1y * v2x;
        
        cumulative_curvature += curvature;
        valid_samples++;
    }
    
    return valid_samples > 0 ? cumulative_curvature / valid_samples : 0.0;
}

// Helper function to calculate total path length
double waypoints_routing::calculatePathLength(const routing::LaneletPath &path)
{
    double total_length = 0.0;
    
    for (const auto &lanelet : path)
    {
        auto points = lanelet.centerline3d();
        for (size_t i = 1; i < points.size(); ++i)
        {
            double dx = points[i].x() - points[i-1].x();
            double dy = points[i].y() - points[i-1].y();
            double dz = points[i].z() - points[i-1].z();
            total_length += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    }
    
    return total_length;
}

// Helper function to calculate remaining path length from a given index
double waypoints_routing::calculateRemainingPathLength(const routing::LaneletPath &path, int start_index)
{
    if (start_index < 0 || start_index >= path.size())
        return 0.0;
    
    double remaining_length = 0.0;
    
    for (size_t i = start_index; i < path.size(); ++i)
    {
        auto points = path[i].centerline3d();
        for (size_t j = 1; j < points.size(); ++j)
        {
            double dx = points[j].x() - points[j-1].x();
            double dy = points[j].y() - points[j-1].y();
            double dz = points[j].z() - points[j-1].z();
            remaining_length += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    }
    
    return remaining_length;
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

bool waypoints_routing::isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet)
{
    // Get centerlines of both lanelets
    auto path_points = path_lanelet.centerline3d();
    auto candidate_points = candidate_lanelet.centerline3d();
    
    if (path_points.size() < 2 || candidate_points.size() < 2)
        return false;
    
    // Calculate the direction vector of the path lanelet
    double path_dx = path_points.back().x() - path_points.front().x();
    double path_dy = path_points.back().y() - path_points.front().y();
    double path_length = std::sqrt(path_dx*path_dx + path_dy*path_dy);
    
    if (path_length < 1e-6)
        return false;
    
    // Normalize the path direction vector
    path_dx /= path_length;
    path_dy /= path_length;
    
    // Calculate the direction vector of the candidate lanelet
    double candidate_dx = candidate_points.back().x() - candidate_points.front().x();
    double candidate_dy = candidate_points.back().y() - candidate_points.front().y();
    double candidate_length = std::sqrt(candidate_dx*candidate_dx + candidate_dy*candidate_dy);
    
    if (candidate_length < 1e-6)
        return false;
    
    // Normalize the candidate direction vector
    candidate_dx /= candidate_length;
    candidate_dy /= candidate_length;
    
    // Calculate the dot product to determine if they're parallel or branching
    double dot_product = path_dx * candidate_dx + path_dy * candidate_dy;
    
    // Calculate the distance between the start points of both lanelets
    double start_dx = candidate_points.front().x() - path_points.front().x();
    double start_dy = candidate_points.front().y() - path_points.front().y();
    double start_distance = std::sqrt(start_dx*start_dx + start_dy*start_dy);
    
    // A lanelet is considered "branching" if:
    // 1. It's not perfectly parallel (dot product < 0.9)
    // 2. OR it's far enough from the path lanelet (> 10m) to be considered a separate branch
    
    // If dot product is very high (> 0.9), they're almost parallel
    if (dot_product > 0.9)
    {
        // If they're close together (< 10m), it's likely a parallel neighbor, not a branch
        if (start_distance < 10.0)
        {
            return false; // This is a parallel neighbor, not a branch
        }
    }
    
    // If dot product is moderate (0.3 to 0.9), they're branching
    if (dot_product > 0.3 && dot_product <= 0.9)
    {
        return true; // This is a branching lanelet
    }
    
    // If dot product is low (< 0.3), they're going in very different directions
    // This could be a sharp turn or branch
    return true; // Consider it a branch
}

// Helper function to count meaningful connections for a candidate lanelet
int waypoints_routing::countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, int current_path_index)
{
    int meaningful_connections = 0;
    int start_connections = 0;  // Connections at the start of the lanelet
    int end_connections = 0;   // Connections at the end of the lanelet
    
    // Get the lanelets that follow and precede the candidate
    auto candidate_following = routingGraph->following(candidate_lanelet, true);
    auto candidate_previous = routingGraph->previous(candidate_lanelet, true);
    
    // NEW: Also get adjacent lanelets (left, right, adjacentLeft, adjacentRight)
    // These represent merging or parallel paths that should be counted as connections
    auto left_lanelet = routingGraph->left(candidate_lanelet);
    auto right_lanelet = routingGraph->right(candidate_lanelet);
    auto adjacent_left = routingGraph->adjacentLeft(candidate_lanelet);
    auto adjacent_right = routingGraph->adjacentRight(candidate_lanelet);
    
    std::vector<lanelet::ConstLanelet> adjacent_lanelets;
    if (left_lanelet) adjacent_lanelets.push_back(*left_lanelet);
    if (right_lanelet) adjacent_lanelets.push_back(*right_lanelet);
    if (adjacent_left) adjacent_lanelets.push_back(*adjacent_left);
    if (adjacent_right) adjacent_lanelets.push_back(*adjacent_right);
    
    // Get candidate lanelet points for endpoint analysis
    auto candidate_points = candidate_lanelet.centerline3d();
    if (candidate_points.empty())
        return 0;
        
    auto candidate_start = candidate_points.front();
    auto candidate_end = candidate_points.back();
    
    // Check if candidate can reach the final destination
    try 
    {
        auto route_to_destination = routingGraph->getRoute(candidate_lanelet, shortestPath.back(), 0);
        if (route_to_destination)
        {
            auto candidate_path = route_to_destination->shortestPath();
            double candidate_route_length = calculatePathLength(candidate_path);
            double main_path_remaining_length = calculateRemainingPathLength(shortestPath, current_path_index);
            
            // Allow routes that are at most 50% longer than the remaining main path
            if (candidate_route_length <= main_path_remaining_length * 1.5)
            {
                meaningful_connections++; // Count destination as a meaningful connection
            }
        }
    }
    catch (...)
    {
        // Destination not reachable
    }
    
    // Count connections to path ahead
    bool connects_to_path_ahead = false;
    if (current_path_index >= 0)
    {
        // Check following connections
        for (const auto &following_ll : candidate_following)
        {
            for (size_t i = current_path_index + 1; i < shortestPath.size(); ++i)
            {
                if (following_ll.id() == shortestPath[i].id())
                {
                    connects_to_path_ahead = true;
                    meaningful_connections++;
                    break;
                }
            }
            if (connects_to_path_ahead) break;
        }
        
        // Check previous connections if not already found
        if (!connects_to_path_ahead)
        {
            for (const auto &previous_ll : candidate_previous)
            {
                for (size_t i = current_path_index + 1; i < shortestPath.size(); ++i)
                {
                    if (previous_ll.id() == shortestPath[i].id())
                    {
                        connects_to_path_ahead = true;
                        meaningful_connections++;
                        break;
                    }
                }
                if (connects_to_path_ahead) break;
            }
        }
    }
    
    // Count additional unique connections (excluding already counted ones)
    std::set<lanelet::Id> counted_lanelets;
    
    // Add destination path lanelets to counted set
    if (meaningful_connections > 0)
    {
        try 
        {
            auto route_to_destination = routingGraph->getRoute(candidate_lanelet, shortestPath.back(), 0);
            if (route_to_destination)
            {
                auto candidate_path = route_to_destination->shortestPath();
                for (const auto &path_ll : candidate_path)
                {
                    counted_lanelets.insert(path_ll.id());
                }
            }
        }
        catch (...) {}
        
        // Add path ahead lanelets to counted set
        if (current_path_index >= 0)
        {
            for (size_t i = current_path_index + 1; i < shortestPath.size(); ++i)
            {
                counted_lanelets.insert(shortestPath[i].id());
            }
        }
    }
    
    // Count additional following connections
    for (const auto &following_ll : candidate_following)
    {
        if (counted_lanelets.find(following_ll.id()) == counted_lanelets.end())
        {
            meaningful_connections++;
            counted_lanelets.insert(following_ll.id());
        }
    }
    
    // Count additional previous connections
    for (const auto &previous_ll : candidate_previous)
    {
        if (counted_lanelets.find(previous_ll.id()) == counted_lanelets.end())
        {
            meaningful_connections++;
            counted_lanelets.insert(previous_ll.id());
        }
    }
    
    // NEW: Count adjacent/merging lanelets as meaningful connections
    // These represent paths that are spatially close and likely merging or parallel
    for (const auto &adjacent_ll : adjacent_lanelets)
    {
        if (counted_lanelets.find(adjacent_ll.id()) == counted_lanelets.end())
        {
            // Check if this adjacent lanelet is part of the main path (high priority connection)
            bool is_part_of_main_path = false;
            for (const auto &main_ll : shortestPath)
            {
                if (adjacent_ll.id() == main_ll.id())
                {
                    is_part_of_main_path = true;
                    break;
                }
            }
            
            // Count adjacent lanelets as connections, especially if they're part of main path
            if (is_part_of_main_path)
            {
                meaningful_connections += 2; // Give higher weight to main path connections
                std::cout << yellow << "      Adjacent lanelet " << adjacent_ll.id() << " is part of main path - counted as 2 connections" << reset << std::endl;
            }
            else
            {
                meaningful_connections += 1; // Regular adjacent connection
                std::cout << yellow << "      Adjacent lanelet " << adjacent_ll.id() << " counted as 1 connection" << reset << std::endl;
            }
            
            counted_lanelets.insert(adjacent_ll.id());
        }
    }
    
    // NEW: Check for connections to blue/orange paths (main path and neighbor lanelets) at START and END
    // This ensures the lanelet connects to valid paths at both endpoints
    
    // First, check connections to main path (blue paths)
    for (const auto &main_ll : shortestPath)
    {
        if (counted_lanelets.find(main_ll.id()) != counted_lanelets.end())
            continue; // Already counted
            
        auto main_points = main_ll.centerline3d();
        if (main_points.empty())
            continue;
            
        auto main_start = main_points.front();
        auto main_end = main_points.back();
        
        // Check connection at START of candidate lanelet
        double start_to_main_start = std::sqrt(
            std::pow(candidate_start.x() - main_start.x(), 2) +
            std::pow(candidate_start.y() - main_start.y(), 2)
        );
        double start_to_main_end = std::sqrt(
            std::pow(candidate_start.x() - main_end.x(), 2) +
            std::pow(candidate_start.y() - main_end.y(), 2)
        );
        
        // Check connection at END of candidate lanelet
        double end_to_main_start = std::sqrt(
            std::pow(candidate_end.x() - main_start.x(), 2) +
            std::pow(candidate_end.y() - main_start.y(), 2)
        );
        double end_to_main_end = std::sqrt(
            std::pow(candidate_end.x() - main_end.x(), 2) +
            std::pow(candidate_end.y() - main_end.y(), 2)
        );
        
        // If candidate connects to main path at start (within 6m - more reasonable)
        if (start_to_main_start < 6.0 || start_to_main_end < 6.0)
        {
            start_connections++;
            meaningful_connections++;
            counted_lanelets.insert(main_ll.id());
            std::cout << yellow << "      Main path lanelet " << main_ll.id() << " connected at START (dist=" 
                      << std::min(start_to_main_start, start_to_main_end) << "m)" << reset << std::endl;
        }
        
        // If candidate connects to main path at end (within 8m - more lenient for end connections)
        if (end_to_main_start < 8.0 || end_to_main_end < 8.0)
        {
            end_connections++;
            if (counted_lanelets.find(main_ll.id()) == counted_lanelets.end())
            {
                meaningful_connections++;
                counted_lanelets.insert(main_ll.id());
            }
            std::cout << yellow << "      Main path lanelet " << main_ll.id() << " connected at END (dist=" 
                      << std::min(end_to_main_start, end_to_main_end) << "m)" << reset << std::endl;
        }
    }
    
    // Also check connections to neighbor lanelets (orange paths) from the same lane
    for (const auto &path_lanelet : shortestPath)
    {
        // Get all lanelets in the same lane (besides the current lanelet)
        ConstLanelets lane_lanelets = routingGraph->besides(path_lanelet);
        
        // Also get adjacent lanelets for this path lanelet
        auto left_lanelet = routingGraph->left(path_lanelet);
        auto right_lanelet = routingGraph->right(path_lanelet);
        auto adjacent_left = routingGraph->adjacentLeft(path_lanelet);
        auto adjacent_right = routingGraph->adjacentRight(path_lanelet);
        
        // Add adjacent lanelets to the neighbor list
        if (left_lanelet) lane_lanelets.push_back(*left_lanelet);
        if (right_lanelet) lane_lanelets.push_back(*right_lanelet);
        if (adjacent_left) lane_lanelets.push_back(*adjacent_left);
        if (adjacent_right) lane_lanelets.push_back(*adjacent_right);
        
        for (const auto &neighbor_ll : lane_lanelets)
        {
            if (counted_lanelets.find(neighbor_ll.id()) != counted_lanelets.end())
                continue; // Already counted
                
            auto neighbor_points = neighbor_ll.centerline3d();
            if (neighbor_points.empty())
                continue;
                
            auto neighbor_start = neighbor_points.front();
            auto neighbor_end = neighbor_points.back();
            
            // Check connection at START of candidate lanelet
            double start_to_neighbor_start = std::sqrt(
                std::pow(candidate_start.x() - neighbor_start.x(), 2) +
                std::pow(candidate_start.y() - neighbor_start.y(), 2)
            );
            double start_to_neighbor_end = std::sqrt(
                std::pow(candidate_start.x() - neighbor_end.x(), 2) +
                std::pow(candidate_start.y() - neighbor_end.y(), 2)
            );
            
            // Check connection at END of candidate lanelet
            double end_to_neighbor_start = std::sqrt(
                std::pow(candidate_end.x() - neighbor_start.x(), 2) +
                std::pow(candidate_end.y() - neighbor_start.y(), 2)
            );
            double end_to_neighbor_end = std::sqrt(
                std::pow(candidate_end.x() - neighbor_end.x(), 2) +
                std::pow(candidate_end.y() - neighbor_end.y(), 2)
            );
            
            // If candidate connects to neighbor at start (within 6m - more reasonable)
            if (start_to_neighbor_start < 6.0 || start_to_neighbor_end < 6.0)
            {
                start_connections++;
                meaningful_connections++;
                counted_lanelets.insert(neighbor_ll.id());
                std::cout << yellow << "      Neighbor lanelet " << neighbor_ll.id() << " connected at START (dist=" 
                          << std::min(start_to_neighbor_start, start_to_neighbor_end) << "m)" << reset << std::endl;
            }
            
            // If candidate connects to neighbor at end (within 8m - more lenient for end connections)
            if (end_to_neighbor_start < 8.0 || end_to_neighbor_end < 8.0)
            {
                end_connections++;
                if (counted_lanelets.find(neighbor_ll.id()) == counted_lanelets.end())
                {
                    meaningful_connections++;
                    counted_lanelets.insert(neighbor_ll.id());
                }
                std::cout << yellow << "      Neighbor lanelet " << neighbor_ll.id() << " connected at END (dist=" 
                          << std::min(end_to_neighbor_start, end_to_neighbor_end) << "m)" << reset << std::endl;
            }
        }
    }
    
    std::cout << yellow << "    Final connection count: " << meaningful_connections 
              << " (start=" << start_connections << ", end=" << end_connections << ")" << reset << std::endl;
    
    // Debug: Show candidate lanelet endpoints
    std::cout << yellow << "    Candidate lanelet " << candidate_lanelet.id() << " endpoints: start=(" 
              << candidate_start.x() << "," << candidate_start.y() << ") end=(" 
              << candidate_end.x() << "," << candidate_end.y() << ")" << reset << std::endl;
    
    // STRICT VALIDATION: Require both start AND end connections for valid curves
    // Valid curves must connect to blue/orange paths at both beginning and end
    if (meaningful_connections < 1)
    {
        std::cout << yellow << "    Lanelet rejected: no meaningful connections found" << reset << std::endl;
        return 0;
    }
    
    // NEW: Require both start AND end connections for valid curves
    if (start_connections == 0)
    {
        std::cout << yellow << "    Lanelet rejected: no connections at START - curves must connect to blue/orange paths at beginning" << reset << std::endl;
        return 0;
    }
    
    if (end_connections == 0)
    {
        std::cout << yellow << "    Lanelet rejected: no connections at END - curves must connect to blue/orange paths at end" << reset << std::endl;
        return 0;
    }
    
    // Accept only if we have connections at both start AND end
    std::cout << yellow << "    Lanelet accepted: has meaningful connections at both start and end (start=" << start_connections 
              << ", end=" << end_connections << ", total=" << meaningful_connections << ")" << reset << std::endl;
    
    return meaningful_connections;
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