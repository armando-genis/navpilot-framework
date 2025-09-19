#include "GlobalPlanner.hpp"

GlobalPlanner::GlobalPlanner(double x_offset, double y_offset, std::string map_path, int start_lanelet_id, int end_lanelet_id)
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
            }
            
        }
    }
}

