#include "GlobalPlanner.hpp"
#include <queue>
#include <algorithm>

GlobalPlanner::GlobalPlanner(double x_offset, double y_offset, std::string map_path, int start_lanelet_id, int end_lanelet_id, double resolution, int close_radius, int close_iters, int outside_value, std::string frame_id)
{
    x_offset_ = x_offset;
    y_offset_ = y_offset;
    map_path_ = map_path;
    start_lanelet_id_ = start_lanelet_id;
    end_lanelet_id_ = end_lanelet_id;

    // Load the lanelet map
    lanelet::Origin origin({49, 8.4});
    lanelet::projection::LocalCartesianProjector projector(origin);
    lanelet::LaneletMapPtr map = lanelet::load(map_path_, projector);

    for (auto &point : map->pointLayer)
    {
        point.x() = point.attribute("local_x").asDouble().value() + x_offset_;
        point.y() = point.attribute("local_y").asDouble().value() + y_offset_;
    }

    map_routing(map);
    occupancy_grid_ready_ = false;

    resolution_ = resolution;
    close_radius_ = close_radius;
    close_iters_ = close_iters;
    outside_value_ = outside_value;
    frame_id_ = frame_id;

    generateOccupancyGrid(map);
    occupancy_grid_ready_ = true;
}

GlobalPlanner::~GlobalPlanner()
{
}

void GlobalPlanner::map_routing(lanelet::LaneletMapPtr &map)
{
    traffic_rules::TrafficRulesPtr trafficRules =
        traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);

    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

    if (routingGraph)
    {
        std::cout << green << "Routing graph built successfully" << reset << std::endl;

        lanelet::ConstLanelet startLanelet = map->laneletLayer.get(start_lanelet_id_);
        lanelet::ConstLanelet endLanelet = map->laneletLayer.get(end_lanelet_id_);

        // Check if the goal lanelet is reachable from the start lanelet
        double maxRoutingCost = 500.0;
        auto reachableSet = routingGraph->reachableSet(startLanelet, maxRoutingCost);
        bool isReachable = std::find_if(reachableSet.begin(), reachableSet.end(),
                                        [&](const lanelet::ConstLanelet &ll)
                                        { return ll.id() == endLanelet.id(); }) != reachableSet.end();

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
                routing::LaneletPath shortestPath = route->shortestPath();

                // Generate waypoints for neighboring lanelets
                generateNeighborWaypoints(map, routingGraph, shortestPath);
            }
            
        }
    }
}


void GlobalPlanner::generateNeighborWaypoints(lanelet::LaneletMapPtr &map, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath)
{
    std::cout << green << "Generating neighbor waypoints for routing path..." << reset << std::endl;
    
    std::set<lanelet::Id> processed_lanelets; // To avoid duplicates
    
    // Clear previous neighbor waypoints
    neighbor_points_.clear();

    int lane_sequence_id = 0;
    int lanelet_id = 0;
    // First, add waypoints from the main routing path
    for (const auto &path_lanelet : shortestPath)
    {
        if (processed_lanelets.find(path_lanelet.id()) != processed_lanelets.end())
            continue;
            
        processed_lanelets.insert(path_lanelet.id());
        lanelet_id = path_lanelet.id();
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

        std::vector<point_struct> main_path_points;
        
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
            
            // Store the point with priority 1 (blue path)
            point_struct waypoint_data;
            waypoint_data.x = point.x();
            waypoint_data.y = point.y();
            waypoint_data.heading = yaw;
            waypoint_data.priority = 1; // Blue path priority
            waypoint_data.lanelet_id = lanelet_id;
            waypoint_data.lane_sequence_id = 0;
            main_path_points.push_back(waypoint_data);

        }

        neighbor_points_.push_back(main_path_points);
    }

    lanelet_id = 0;
    lane_sequence_id = 1;
    
    // Collect all neighbor lanelets first, then group them by subsequential connections
    std::vector<lanelet::ConstLanelet> all_neighbor_lanelets;
    for (const auto &path_lanelet : shortestPath)
    {
        // Get all lanelets in the same lane (besides the current lanelet)
        ConstLanelets lane_lanelets = routingGraph->besides(path_lanelet);
        
        for (const auto &lanelet : lane_lanelets)
        {
            // Skip the main path lanelet and already processed lanelets
            if (lanelet.id() == path_lanelet.id() || processed_lanelets.find(lanelet.id()) != processed_lanelets.end())
                continue;
            all_neighbor_lanelets.push_back(lanelet);
        }
    }
    
    // Group subsequential lanelets together
    std::vector<std::vector<lanelet::ConstLanelet>> neighbor_groups;
    std::set<lanelet::Id> grouped_lanelets;
    
    for (const auto &lanelet : all_neighbor_lanelets)
    {
        if (grouped_lanelets.find(lanelet.id()) != grouped_lanelets.end())
            continue;
            
        std::vector<lanelet::ConstLanelet> current_group;
        std::queue<lanelet::ConstLanelet> to_process;
        to_process.push(lanelet);
        
        while (!to_process.empty())
        {
            auto current_lanelet = to_process.front();
            to_process.pop();
            
            if (grouped_lanelets.find(current_lanelet.id()) != grouped_lanelets.end())
                continue;
                
            grouped_lanelets.insert(current_lanelet.id());
            current_group.push_back(current_lanelet);
            
            // Find connected lanelets (following and previous)
            auto following = routingGraph->following(current_lanelet, true);
            auto previous = routingGraph->previous(current_lanelet, true);
            
            // Add following lanelets that are also neighbors
            for (const auto &follow_lanelet : following)
            {
                if (grouped_lanelets.find(follow_lanelet.id()) == grouped_lanelets.end() &&
                    std::find(all_neighbor_lanelets.begin(), all_neighbor_lanelets.end(), follow_lanelet) != all_neighbor_lanelets.end())
                {
                    to_process.push(follow_lanelet);
                }
            }
            
            // Add previous lanelets that are also neighbors
            for (const auto &prev_lanelet : previous)
            {
                if (grouped_lanelets.find(prev_lanelet.id()) == grouped_lanelets.end() &&
                    std::find(all_neighbor_lanelets.begin(), all_neighbor_lanelets.end(), prev_lanelet) != all_neighbor_lanelets.end())
                {
                    to_process.push(prev_lanelet);
                }
            }
        }
        
        if (!current_group.empty())
        {
            neighbor_groups.push_back(current_group);
        }
    }
    
    // Process each group with the same lane_sequence_id
    for (const auto &group : neighbor_groups)
    {
        for (const auto &lanelet : group)
        {
            processed_lanelets.insert(lanelet.id());
            lanelet_id = lanelet.id();
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

            std::vector<point_struct> neighbor_lanelet_points;
            
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
                
                // Store the point with priority 2 (orange path)
                point_struct waypoint_data;
                waypoint_data.x = point.x();
                waypoint_data.y = point.y();
                waypoint_data.heading = yaw;
                waypoint_data.priority = 2; // Orange path priority
                waypoint_data.lanelet_id = lanelet_id;
                waypoint_data.lane_sequence_id = lane_sequence_id; // Same ID for all lanelets in the group
                neighbor_lanelet_points.push_back(waypoint_data);
            }

            neighbor_points_.push_back(neighbor_lanelet_points);
        }
        lane_sequence_id++; // Increment after each group (not after each individual lanelet)
    }

    // std::cout << blue << "Finding lanelets that branch off through curves..." << reset << std::endl;
    lanelet_id = 0;
    for (const auto &path_lanelet : shortestPath)
    {
        // Get lanelets that follow this path lanelet (branches, curves, etc.)
        auto following_lanelets = routingGraph->following(path_lanelet, true); // withLaneChanges = true
        
        // std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has " << following_lanelets.size() << " following lanelets" << reset << std::endl;
        
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
            
            // std::cout << blue << "  Evaluating following lanelet " << following_lanelet.id() << " for trajectory compatibility..." << reset << std::endl;
            
            //  More strict direction and trajectory checking
            if (!isCompatibleTrajectory(path_lanelet, following_lanelet, routingGraph, shortestPath, map))
            {
                // std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out due to incompatible trajectory" << reset << std::endl;
                continue;
            }
            
            // Check if this is actually a branching lanelet (not just a parallel neighbor)
            if (!isBranchingLanelet(path_lanelet, following_lanelet))
            {
                // std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out - not a branching lanelet" << reset << std::endl;
                continue;
            }
            
            // Check if this lanelet is not beyond the target
            if (isBeyondTarget(following_lanelet, shortestPath))
            {
                // std::cout << yellow << "  Following lanelet " << following_lanelet.id() << " filtered out due to being beyond target" << reset << std::endl;
                continue;
            }
            
            // std::cout << green << "  Adding following lanelet " << following_lanelet.id() << " as purple arrow" << reset << std::endl;
                
            processed_lanelets.insert(following_lanelet.id());
            
            auto points = following_lanelet.centerline3d();
            
            if (points.empty())
                continue;
                
            // Generate waypoints at regular intervals along the lanelet
            std::vector<lanelet::ConstPoint3d> waypoints;
            
            // Always add the first point
            waypoints.push_back(points[0]);

            lanelet_id = following_lanelet.id();
            
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

            std::vector<point_struct> branching_lanelet_points;
            
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
                
                // Store the point with priority 3 (purple path)
                point_struct waypoint_data;
                waypoint_data.x = point.x();
                waypoint_data.y = point.y();
                waypoint_data.heading = yaw;
                waypoint_data.priority = 3; // Purple path priority
                waypoint_data.lanelet_id = lanelet_id;
                waypoint_data.lane_sequence_id = lane_sequence_id;
                branching_lanelet_points.push_back(waypoint_data);
            }

            neighbor_points_.push_back(branching_lanelet_points);
            lane_sequence_id++;
        }
    }
    
    // std::cout << blue << "Finding lanelets connected through adjacency..." << reset << std::endl;
    lanelet_id = 0;
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
        
        // std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has " << adjacent_lanelets.size() << " adjacent lanelets" << reset << std::endl;
        
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
            
            // std::cout << blue << "  Evaluating adjacent lanelet " << adjacent_lanelet.id() << " for trajectory compatibility..." << reset << std::endl;
            
            // IMPROVED: Use the same strict trajectory checking for adjacent lanelets
            if (!isCompatibleTrajectory(path_lanelet, adjacent_lanelet, routingGraph, shortestPath, map))
            {
                // std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id() << " filtered out due to incompatible trajectory" << reset << std::endl;
                continue;
            }
            
            // Check if this lanelet is not beyond the target
            if (isBeyondTarget(adjacent_lanelet, shortestPath))
            {
                // std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id() << " filtered out due to being beyond target" << reset << std::endl;
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
            lanelet_id = adjacent_lanelet.id();
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

            std::vector<point_struct> adjacent_lanelet_points;
            
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
                
                // Store the point with priority 4 (purple path)
                point_struct waypoint_data;
                waypoint_data.x = point.x();
                waypoint_data.y = point.y();
                waypoint_data.heading = yaw;
                waypoint_data.priority = 4; // Purple path priority
                waypoint_data.lanelet_id = lanelet_id;
                waypoint_data.lane_sequence_id = lane_sequence_id; 
                adjacent_lanelet_points.push_back(waypoint_data);
            }
            neighbor_points_.push_back(adjacent_lanelet_points);
            lane_sequence_id++; 
        }
    }

    all_waypoints_ = getAllWaypointsStruct();
    std::cout << green << "Neighbor points generated with " << neighbor_points_.size() << " points" << reset << std::endl;
    std::cout << green << "All neighbor points generated with " << all_waypoints_.size() << " points" << reset << std::endl;
}

std::vector<point_struct> GlobalPlanner::getAllAllWaypointsStruct()
{
    return all_waypoints_;
}

bool GlobalPlanner::isBeyondTarget(const lanelet::ConstLanelet &lanelet, const routing::LaneletPath &shortestPath)
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

bool GlobalPlanner::isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet)
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


bool GlobalPlanner::isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet, const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, lanelet::LaneletMapPtr &map)
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
    

    if (overall_angle > max_allowed_angle)
    {
        // std::cout << yellow << "    Trajectory rejected: poor overall direction alignment" << reset << std::endl;
        return false;
    }
    
    // Check if this is a consistent diverging curve by analyzing the path end direction
    // Get the direction at the end of the path lanelet
    std::vector<lanelet::ConstPoint3d> path_vector(path_points.begin(), path_points.end());
    std::vector<lanelet::ConstPoint3d> candidate_vector(candidate_points.begin(), candidate_points.end());
    
    auto path_end_direction = getEndDirection(path_vector);
    auto candidate_start_direction = getStartDirection(candidate_vector);
    
    double transition_dot = path_end_direction.first * candidate_start_direction.first + 
                           path_end_direction.second * candidate_start_direction.second;
    double transition_angle = std::acos(std::max(-1.0, std::min(1.0, transition_dot))) * 180.0 / M_PI;
    
    // std::cout << yellow << "    Transition angle: " << transition_angle << "°" << reset << std::endl;
    
    // Check for sharp direction changes at the connection point
    if (transition_angle > 90.0) // Very sharp transition
    {
        // std::cout << yellow << "    Trajectory rejected: sharp transition at connection" << reset << std::endl;
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
        
        if (path_idx + 1 >= static_cast<int>(path_points.size()) || candidate_idx + 1 >= static_cast<int>(candidate_points.size()))
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
    
    // Allow curves to have higher average deviation
    double max_avg_deviation = is_direct_continuation ? 50.0 : 35.0; // Increased limits
    
    if (avg_deviation_deg > max_avg_deviation)
    {
        // std::cout << yellow << "    Trajectory rejected: excessive average deviation" << reset << std::endl;
        return false;
    }
    
    // 6. Check if candidate connects to valid lanelets
    double avg_cross = cumulative_cross / valid_samples;
    
    // std::cout << yellow << "    Turn analysis: avg_cross=" << avg_cross << reset << std::endl;
    
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
    int meaningful_connections = countMeaningfulConnections(candidate_lanelet, routingGraph, shortestPath, current_path_index, map);
    
    // std::cout << yellow << "    Connectivity analysis: meaningful_connections=" << meaningful_connections << reset << std::endl;
    
    // Require at least 2 meaningful connections AND both start/end connections for valid curves/paths
    // This ensures lanelets connect to multiple blue/orange paths and aren't dead-ends
    if (meaningful_connections < 2)
    {
        // std::cout << yellow << "    Trajectory rejected: insufficient meaningful connections (" << meaningful_connections 
        //           << " < 2). Valid curves/paths must connect to at least 2 different lanelets or destinations." << reset << std::endl;
        return false;
    }
    
    // Additional check: If it connects but has very strong divergent turning, still be suspicious
    if (std::abs(avg_cross) > 0.8 && meaningful_connections < 2)
    {
        // Very strong turning + very limited connections = suspicious
        // std::cout << yellow << "    Trajectory rejected: very strong divergent turning with very limited connections" << reset << std::endl;
        return false;
    }
    
    // Keep a safety threshold for extremely sharp turns
    if (std::abs(avg_cross) > 1.2)
    {
        // std::cout << yellow << "    Trajectory rejected: extremely sharp turning" << reset << std::endl;
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
    

    // Much more lenient for direct continuations, especially curves
    double max_divergence_factor = is_direct_continuation ? 1.0 : 0.4; // Very lenient for continuations
    double max_lanelet_length = std::max(path_length, candidate_length);
    
    if (end_distance > start_distance + max_lanelet_length * max_divergence_factor)
    {
        // std::cout << yellow << "    Trajectory rejected: excessive divergence" << reset << std::endl;
        return false;
    }
    
    // std::cout << green << "    Trajectory accepted: compatible path (continuation=" 
    //           << (is_direct_continuation ? "YES" : "NO") << ")" << reset << std::endl;

    return true;
}

// Helper function to count meaningful connections for a candidate lanelet
int GlobalPlanner::countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet, routing::RoutingGraphUPtr &routingGraph, const routing::LaneletPath &shortestPath, int current_path_index, lanelet::LaneletMapPtr &map)
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
    
    // Count adjacent/merging lanelets as meaningful connections
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
                // std::cout << yellow << "      Adjacent lanelet " << adjacent_ll.id() << " is part of main path - counted as 2 connections" << reset << std::endl;
            }
            else
            {
                meaningful_connections += 1; // Regular adjacent connection
                // std::cout << yellow << "      Adjacent lanelet " << adjacent_ll.id() << " counted as 1 connection" << reset << std::endl;
            }
            
            counted_lanelets.insert(adjacent_ll.id());
        }
    }
    
    // Check for connections to blue/orange paths (main path and neighbor lanelets) at START and END
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
            // std::cout << yellow << "      Main path lanelet " << main_ll.id() << " connected at START (dist=" 
            //           << std::min(start_to_main_start, start_to_main_end) << "m)" << reset << std::endl;
        }
        
        // DEBUG: Show all distances for main path lanelets
        // std::cout << yellow << "      Main path lanelet " << main_ll.id() << " END distances: to_start=" << end_to_main_start 
        //           << "m, to_end=" << end_to_main_end << "m" << reset << std::endl;
        
        // If candidate connects to main path at end (within 5m)
        if (end_to_main_start < 5.0 || end_to_main_end < 5.0)
        {
            end_connections++;
            if (counted_lanelets.find(main_ll.id()) == counted_lanelets.end())
            {
                meaningful_connections++;
                counted_lanelets.insert(main_ll.id());
            }
            // std::cout << yellow << "      Main path lanelet " << main_ll.id() << " connected at END (dist=" 
            //           << std::min(end_to_main_start, end_to_main_end) << "m)" << reset << std::endl;
        }
    }
    
    // Also check connections to lanelets that are reachable from the main path
    // This includes lanelets that are following, previous, or adjacent to main path lanelets
    std::set<lanelet::Id> reachable_lanelets;
    for (const auto &main_ll : shortestPath)
    {
        // Get following lanelets
        auto following = routingGraph->following(main_ll, true);
        for (const auto &follower : following)
        {
            reachable_lanelets.insert(follower.id());
        }
        
        // Get previous lanelets
        auto previous = routingGraph->previous(main_ll, true);
        for (const auto &prev : previous)
        {
            reachable_lanelets.insert(prev.id());
        }
        
        // Get adjacent lanelets
        auto left = routingGraph->left(main_ll);
        auto right = routingGraph->right(main_ll);
        auto adj_left = routingGraph->adjacentLeft(main_ll);
        auto adj_right = routingGraph->adjacentRight(main_ll);
        
        if (left) reachable_lanelets.insert(left->id());
        if (right) reachable_lanelets.insert(right->id());
        if (adj_left) reachable_lanelets.insert(adj_left->id());
        if (adj_right) reachable_lanelets.insert(adj_right->id());
    }
    
    // Check connections to reachable lanelets
    for (const auto &reachable_id : reachable_lanelets)
    {
        if (counted_lanelets.find(reachable_id) != counted_lanelets.end())
            continue; // Already counted
            
        // Skip self-connections - a lanelet should not count as connecting to itself
        if (reachable_id == candidate_lanelet.id())
        {
            // std::cout << yellow << "      Skipping self-connection for lanelet " << reachable_id << reset << std::endl;
            continue;
        }
            
        // Get the lanelet from the map
        auto reachable_lanelet = map->laneletLayer.get(reachable_id);
        auto reachable_points = reachable_lanelet.centerline3d();
        if (reachable_points.empty())
            continue;
            
        auto reachable_start = reachable_points.front();
        auto reachable_end = reachable_points.back();
        
        // Check connection at START of candidate lanelet
        double start_to_reachable_start = std::sqrt(
            std::pow(candidate_start.x() - reachable_start.x(), 2) +
            std::pow(candidate_start.y() - reachable_start.y(), 2)
        );
        double start_to_reachable_end = std::sqrt(
            std::pow(candidate_start.x() - reachable_end.x(), 2) +
            std::pow(candidate_start.y() - reachable_end.y(), 2)
        );
        
        // Check connection at END of candidate lanelet
        double end_to_reachable_start = std::sqrt(
            std::pow(candidate_end.x() - reachable_start.x(), 2) +
            std::pow(candidate_end.y() - reachable_start.y(), 2)
        );
        double end_to_reachable_end = std::sqrt(
            std::pow(candidate_end.x() - reachable_end.x(), 2) +
            std::pow(candidate_end.y() - reachable_end.y(), 2)
        );
        
        // If candidate connects to reachable lanelet at start (within 6m)
        if (start_to_reachable_start < 6.0 || start_to_reachable_end < 6.0)
        {
            start_connections++;
            meaningful_connections++;
            counted_lanelets.insert(reachable_id);
            // std::cout << yellow << "      Reachable lanelet " << reachable_id << " connected at START (dist=" 
            //           << std::min(start_to_reachable_start, start_to_reachable_end) << "m)" << reset << std::endl;
        }
        
        
        // If candidate connects to reachable lanelet at end (within 5m)
        if (end_to_reachable_start < 5.0 || end_to_reachable_end < 5.0)
        {
            end_connections++;
            if (counted_lanelets.find(reachable_id) == counted_lanelets.end())
            {
                meaningful_connections++;
                counted_lanelets.insert(reachable_id);
            }
            // std::cout << yellow << "      Reachable lanelet " << reachable_id << " connected at END (dist=" 
            //           << std::min(end_to_reachable_start, end_to_reachable_end) << "m)" << reset << std::endl;
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
                // std::cout << yellow << "      Neighbor lanelet " << neighbor_ll.id() << " connected at START (dist=" 
                //           << std::min(start_to_neighbor_start, start_to_neighbor_end) << "m)" << reset << std::endl;
            }
            
            // Show all distances for neighbor lanelets
            // std::cout << yellow << "      Neighbor lanelet " << neighbor_ll.id() << " END distances: to_start=" << end_to_neighbor_start 
            //           << "m, to_end=" << end_to_neighbor_end << "m" << reset << std::endl;
            
            // If candidate connects to neighbor at end (within 5m)
            if (end_to_neighbor_start < 5.0 || end_to_neighbor_end < 5.0)
            {
                end_connections++;
                if (counted_lanelets.find(neighbor_ll.id()) == counted_lanelets.end())
                {
                    meaningful_connections++;
                    counted_lanelets.insert(neighbor_ll.id());
                }
                // std::cout << yellow << "      Neighbor lanelet " << neighbor_ll.id() << " connected at END (dist=" 
                //           << std::min(end_to_neighbor_start, end_to_neighbor_end) << "m)" << reset << std::endl;
            }
        }
    }
        
    // Require both start AND end connections for valid curves
    // Valid curves must connect to blue/orange paths at both beginning and end
    if (meaningful_connections < 1)
    {
        // std::cout << yellow << "    Lanelet rejected: no meaningful connections found" << reset << std::endl;
        return 0;
    }
    
    // Require both start AND end connections for valid curves
    if (start_connections == 0)
    {
        // std::cout << yellow << "    Lanelet rejected: no connections at START - curves must connect to blue/orange paths at beginning" << reset << std::endl;
        return 0;
    }
    
    if (end_connections == 0)
    {
        // std::cout << yellow << "    Lanelet rejected: no connections at END - curves must connect to blue/orange paths at end" << reset << std::endl;
        return 0;
    }
    
    // Check if end connections are to meaningful paths (not just other candidate lanelets)
    // Count only end connections to main path lanelets and neighbor lanelets, not to other candidate lanelets
    int meaningful_end_connections = 0;
    
    // Check end connections to main path lanelets
    for (const auto &main_ll : shortestPath)
    {
        auto main_points = main_ll.centerline3d();
        if (main_points.empty())
            continue;
            
        auto main_start = main_points.front();
        auto main_end = main_points.back();
        
        double end_to_main_start = std::sqrt(
            std::pow(candidate_end.x() - main_start.x(), 2) +
            std::pow(candidate_end.y() - main_start.y(), 2)
        );
        double end_to_main_end = std::sqrt(
            std::pow(candidate_end.x() - main_end.x(), 2) +
            std::pow(candidate_end.y() - main_end.y(), 2)
        );
        
        if (end_to_main_start < 15.0 || end_to_main_end < 15.0)
        {
            meaningful_end_connections++;
        }
    }
    
    // Check end connections to neighbor lanelets
    for (const auto &path_lanelet : shortestPath)
    {
        ConstLanelets lane_lanelets = routingGraph->besides(path_lanelet);
        
        auto left_lanelet = routingGraph->left(path_lanelet);
        auto right_lanelet = routingGraph->right(path_lanelet);
        auto adjacent_left = routingGraph->adjacentLeft(path_lanelet);
        auto adjacent_right = routingGraph->adjacentRight(path_lanelet);
        
        if (left_lanelet) lane_lanelets.push_back(*left_lanelet);
        if (right_lanelet) lane_lanelets.push_back(*right_lanelet);
        if (adjacent_left) lane_lanelets.push_back(*adjacent_left);
        if (adjacent_right) lane_lanelets.push_back(*adjacent_right);
        
        for (const auto &neighbor_ll : lane_lanelets)
        {
            auto neighbor_points = neighbor_ll.centerline3d();
            if (neighbor_points.empty())
                continue;
                
            auto neighbor_start = neighbor_points.front();
            auto neighbor_end = neighbor_points.back();
            
            double end_to_neighbor_start = std::sqrt(
                std::pow(candidate_end.x() - neighbor_start.x(), 2) +
                std::pow(candidate_end.y() - neighbor_start.y(), 2)
            );
            double end_to_neighbor_end = std::sqrt(
                std::pow(candidate_end.x() - neighbor_end.x(), 2) +
                std::pow(candidate_end.y() - neighbor_end.y(), 2)
            );
            
            if (end_to_neighbor_start < 15.0 || end_to_neighbor_end < 15.0)
            {
                meaningful_end_connections++;
            }
        }
    }
    
    // std::cout << yellow << "    End connection analysis: total_end_connections=" << end_connections 
    //           << ", meaningful_end_connections=" << meaningful_end_connections << reset << std::endl;
    
    // Require at least one meaningful end connection (to main path or neighbor lanelets)
    if (meaningful_end_connections == 0)
    {
        // std::cout << yellow << "    Lanelet rejected: no meaningful end connections - curves must connect to main path or neighbor lanelets at end" << reset << std::endl;
        return 0;
    }
    
    // Accept only if we have connections at both start AND meaningful end connections
    // std::cout << yellow << "    Lanelet accepted: has meaningful connections at both start and end (start=" << start_connections 
    //           << ", meaningful_end=" << meaningful_end_connections << ", total=" << meaningful_connections << ")" << reset << std::endl;
    
    return meaningful_connections;
}

// Helper function to get direction at end of path
std::pair<double, double> GlobalPlanner::getEndDirection(const std::vector<lanelet::ConstPoint3d> &points)
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
std::pair<double, double> GlobalPlanner::getStartDirection(const std::vector<lanelet::ConstPoint3d> &points)
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

// Helper function to calculate total path length
double GlobalPlanner::calculatePathLength(const routing::LaneletPath &path)
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
double GlobalPlanner::calculateRemainingPathLength(const routing::LaneletPath &path, int start_index)
{
    if (start_index < 0 || start_index >= static_cast<int>(path.size()))
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

std::vector<point_struct> GlobalPlanner::getAllWaypointsStruct() const
{
    std::vector<point_struct> filtered_points;
    
    for (const auto &lanelet_points : neighbor_points_)
    {
        for (const auto &point : lanelet_points)
        {
            filtered_points.push_back(point);
        }
    }
    
    return filtered_points;
}


// occupancy grid helper functions

void GlobalPlanner::generateOccupancyGrid(lanelet::LaneletMapPtr &t_map)
{
  std::cout << "Generating occupancy grid from lanelets..." << std::endl;
  
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
    std::cout << "Invalid bounds computed; skipping occupancy grid generation." << std::endl;
    return;
  }

  // 2) Calculate grid dimensions
  int width = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
  int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;
  width = std::max(1, width);
  height = std::max(1, height);

  std::cout << "Grid dimensions: " << width << "x" << height << ", resolution: " << resolution_ << std::endl;

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
  occupancy_grid_.header.stamp = rclcpp::Clock().now();
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

  std::cout << "Occupancy grid generated successfully!" << std::endl;
}

void GlobalPlanner::worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const
{
  gx = static_cast<int>(std::floor((wx - min_x) / resolution_));
  gy = static_cast<int>(std::floor((wy - min_y) / resolution_));
}

void GlobalPlanner::drawLine(int x0, int y0, int x1, int y1, int width, int height,
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

void GlobalPlanner::morphClose(std::vector<int8_t> &data, int width, int height, int radius, int iters) const
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

void GlobalPlanner::fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points, int width, int height,
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

nav_msgs::msg::OccupancyGrid GlobalPlanner::getOccupancyGrid()
{
    return occupancy_grid_;
}

bool GlobalPlanner::isOccupancyGridReady()
{
    return occupancy_grid_ready_;
}