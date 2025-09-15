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

void waypoints_routing::publishWaypoints()
{
    waypoints_publisher_->publish(graph_waypoint_markers);
    if (show_full_graph)
    {
        full_graph_publisher_->publish(full_graph_markers);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<waypoints_routing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}