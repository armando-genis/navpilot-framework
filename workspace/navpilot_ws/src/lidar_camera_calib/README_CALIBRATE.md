# Lidar-Camera Calibration Guide

This guide explains how to set up and use the multisensor calibration framework for calibrating lidar and camera sensors. This code is based on the [multisensor_calibration repository](https://fraunhoferiosb.github.io/multisensor_calibration/) but modified for ROS2 Humble.

## Overview

The calibration process involves:

1. Creating a calibration board with ArUco markers
2. Installing dependencies and building the calibration package
3. Running the calibration procedure

---

## 1. Create Calibration Board

Before starting the calibration, you need to generate a calibration board. The board includes:

- **4 ArUco markers** (IDs: 1, 2, 3, 4) positioned at the corners
- **3 circular cutouts** arranged asymmetrically around the center
- Default dimensions: 1.2m × 0.6m

### Prerequisites

Install Python dependencies for board generation:

```bash
pip3 install opencv-python numpy pillow
```

### Generate the Board

Run the calibration board generator script:

```bash
cd /navpilot_ws/src/lidar_camera_calib
python3 calibration_board_withparam.py
```

### Output Files

The script generates the following files in the current directory:

- **`calibration_board.png`** - PNG image of the board (for preview)
- **`calibration_board.pdf`** - PDF file ready for printing (1:1 scale, 1mm = 1px)
- **`calibration_board.svg`** - Vectorized SVG format (for high-quality printing)
- **`calibration_board_config.yaml`** - Configuration file with board specifications (used by the calibration framework)

### Customizing Board Parameters

You can modify the board dimensions and marker properties by editing the parameters at the top of `calibration_board_withparam.py`:

- `board_width_m` / `board_height_m` - Board dimensions in meters
- `marker_edge_m` - ArUco marker size in meters
- `circle_radius_m` - Radius of circular cutouts in meters
- `offset_corner_m` - Offset of markers from board corners
- `offset_center_m` - Offset of circular cutouts from center

### Printing the Board

1. Print the **PDF** file at 100% scale (no scaling)
2. Ensure the board is printed on a rigid, flat surface
3. Verify the dimensions match the specifications (1.2m × 0.6m by default)
4. The board should be mounted on a flat, stable surface for calibration

---

## 2. Install Dependencies

### Install rosdep (if not already installed)

```bash
sudo apt update
sudo apt install -y python3-rosdep
```

### Initialize rosdep (only once per machine)

```bash
sudo rosdep init
rosdep update
```

### Install Workspace Dependencies

```bash
cd /workspace/navpilot_ws
rosdep install --from-paths src --ignore-src -r -y

sudo apt update
sudo apt install -y libtinyxml2-dev
```

---

## 3. Build the Calibration Package

Build the multisensor calibration package:

```bash
cd /workspace/navpilot_ws
MAKEFLAGS='-j8' colcon build \
  --symlink-install \
  --packages-up-to multisensor_calibration \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the workspace:

```bash
source install/setup.bash
```

---

## 4. Run Calibration

Launch the calibration example:

```bash
ros2 launch multisensor_calibration example_velodyne.py
```

**Note:** Make sure to configure the launch file with your specific sensor topics and calibration board configuration file path before running.

---