# Waypoints Routing Node

This ROS2 node provides lanelet-based routing and visualization capabilities for autonomous navigation.

## Features

- **Full Graph Visualization**: Visualizes the entire road network with directional arrows showing street directions
- **Routing**: Finds the shortest path between two lanelets
- **Waypoint Generation**: Extracts three waypoints per lanelet (start, middle, end) for comprehensive road representation

## Topics

- `/waypoints_routing` (visualization_msgs/msg/MarkerArray): Route-specific waypoints
- `/full_graph` (visualization_msgs/msg/MarkerArray): Complete road network visualization

## Parameters

- `map_path` (string): Path to the OSM map file
- `start_lanelet_id` (int): ID of the starting lanelet for routing
- `end_lanelet_id` (int): ID of the destination lanelet for routing
- `show_full_graph` (bool): Whether to display the full road network graph
- `waypoint_interval` (double): Distance between waypoints in meters (default: 3.0)

## Usage

### Launch the node:

```bash
ros2 launch waypoints_routing waypoints.launch.py
```

### Run with custom parameters:

```bash
ros2 run waypoints_routing waypoints_routing_node --ros-args -p map_path:=/path/to/your/map.osm -p start_lanelet_id:=123 -p end_lanelet_id:=456 -p show_full_graph:=true -p waypoint_interval:=5.0
```

## Visualization

The node publishes two types of markers:

1. **Full Graph Markers** (`/full_graph`):

   - Arrow markers showing direction of each street
   - Color-coded by lanelet ID for easy identification
   - Waypoints generated at regular intervals (configurable)
   - Only processes road lanelets (excludes crosswalks, sidewalks, etc.)

2. **Routing Markers** (`/waypoints_routing`):
   - Blue sphere markers showing the specific route path
   - Only visible when a valid route is found

## Configuration

Edit `config/params.yaml` to set default parameters:

```yaml
waypoints_routing_node:
  ros__parameters:
    map_path: "/path/to/your/map.osm"
    start_lanelet_id: 0
    end_lanelet_id: 0
    show_full_graph: true
    waypoint_interval: 3.0
```

## Dependencies

- lanelet2_core
- lanelet2_io
- lanelet2_routing
- lanelet2_traffic_rules
- lanelet2_projection
- visualization_msgs
- tf2
