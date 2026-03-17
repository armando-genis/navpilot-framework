# tcp_gateway — Bidirectional Jetson-to-Jetson TCP Bridge

## Overview

One package, one binary, two roles — configured entirely through YAML.

```
Jetson-1 (SERVER)                        Jetson-2 (CLIENT)
─────────────────────────────────────────────────────────
/velodyne_points ──┐                  ┌── /velodyne_points
/tf ───────────────┼──► TCP port ────►┼── /tf
                   │    5002          │
/marker_array ◄────┼◄── TCP port ─────┼◄─ /marker_array
/camera/image_raw ◄┘                  └── /camera/image_raw
```

## Wire Protocol

```
 0       4        5      6       8           12
 ┌───────┬────────┬──────┬───────┬───────────────────────┐
 │ MAGIC │ topic  │  rsv │  rsv  │   payload_len (BE u32) │ ... payload (CDR) ...
 │  u32  │  _id   │      │       │                        │
 └───────┴────────┴──────┴───────┴───────────────────────┘
```

| topic_id | Message type |
|----------|-------------|
| 1 | `sensor_msgs/PointCloud2` |
| 2 | `tf2_msgs/TFMessage` |
| 3 | `visualization_msgs/MarkerArray` |
| 4 | `sensor_msgs/Image` |

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/tcp_gateway .
cd ~/ros2_ws
colcon build --packages-select tcp_gateway
source install/setup.bash
```

## Configuration

Edit the YAML files in `config/` before launching.

### `config/jetson1_server.yaml`
```yaml
/**:
  ros__parameters:
    server: true          # This is the server (Jetson-1)
    port: 5002
    lidar_topic:      "/velodyne_points"
    tf_topic:         "/tf"
    marker_arr_topic: "/marker_array"
    image_topic:      "/camera/image_raw"
```

### `config/jetson2_client.yaml`
```yaml
/**:
  ros__parameters:
    server: false         # This is the client (Jetson-2)
    remote_ip: "192.168.1.1"   # ← IP of Jetson-1
    port: 5002
    lidar_topic:      "/velodyne_points"
    marker_arr_topic: "/marker_array"
    image_topic:      "/camera/image_raw"
```

## Launch

**Jetson-1 (server — start this first):**
```bash
ros2 launch tcp_gateway jetson1_server.launch.py
```

**Jetson-2 (client):**
```bash
ros2 launch tcp_gateway jetson2_client.launch.py
```

Or with overrides:
```bash
ros2 run tcp_gateway tcp_gateway_node \
  --ros-args -p server:=false \
             -p remote_ip:=192.168.1.1 \
             -p port:=5002 \
             -p lidar_topic:=/velodyne_points \
             -p tf_topic:=/tf \
             -p marker_arr_topic:=/marker_array \
             -p image_topic:=/camera/image_raw
```

## Design notes

- **Same node binary** on both Jetsons — role is set by `server: true/false`.
- **PointCloud2 and Image** use drop-old-if-unsent logic (only the latest frame
  is sent) to prevent queue build-up over slow links.
- **TF and MarkerArray** are queued reliably (no drops).
- Automatic reconnection: the client retries every second; the server re-accepts
  after a disconnect.
- `TCP_NODELAY` is set to minimize latency.
- Receiving and sending run in separate threads; the ROS callbacks never block
  waiting for the network.
