# LaneMapping-ACC

ROS2 Humble package for road segmentation and 3D mapping with Isaac ROS Nvblox.

## Overview

This package extracts road regions (blue color) from a segmentation mask, applies the mask to depth images, and feeds the masked depth to Isaac ROS Nvblox for 3D reconstruction of road surfaces only.

## Features

- **Blue road extraction** from `/segmentation/color_mask` with configurable color thresholds
- **Depth masking** to keep only road regions for 3D reconstruction
- **Synthetic CameraInfo generation** with configurable intrinsics
- **Nvblox integration** in static reconstruction mode (geometry only, no color)

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/segmentation/color_mask` | sensor_msgs/Image | Input RGB segmentation mask |
| `/camera/depth_image` | sensor_msgs/Image | Input depth image from RealSense D435 |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/segmentation/road_mask` | sensor_msgs/Image | Binary road mask (mono8) |
| `/segmentation/color_mask/road/image` | sensor_msgs/Image | Visualization of extracted road |
| `/nvblox/depth/image` | sensor_msgs/Image | Masked depth for nvblox |
| `/nvblox/depth/camera_info` | sensor_msgs/CameraInfo | Camera intrinsics |

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select lane_mapping_acc
source install/setup.bash
```

## Usage

### Run segmentation pipeline only
```bash
ros2 launch lane_mapping_acc road_segmentation.launch.py
```

### Run complete pipeline with Nvblox
```bash
ros2 run tf2_ros static_transform_publisher     --x 0.095 --y 0.032 --z 0.172     --roll -1.5708 --pitch 0 --yaw -1.5708     --frame-id base_link     --child-frame-id camera_depth_optical_frame

ros2 run qcar2_laneseg_acc color_segmentation_node.py --ros-args -p roi_height_ratio:=0.2


ros2 launch lane_mapping_acc cartographer_mapping.launch.py

ros2 launch lane_mapping_acc nvblox_road_mapping.launch.py use_sim_time:=true

```

### Custom configuration
```bash
ros2 launch lane_mapping_acc nvblox_road_mapping.launch.py \
    segmentation_config:=/path/to/custom_segmentation.yaml \
    nvblox_config:=/path/to/custom_nvblox.yaml \
    global_frame:=map
```

## Configuration

### `config/segmentation_params.yaml`
- Blue color thresholds (RGB) and tolerance
- Input/output topic names
- Camera intrinsics (fx, fy, cx, cy)
- Synchronization parameters

### `config/nvblox_static.yaml`
- Nvblox parameters for static TSDF reconstruction
- ESDF configuration for navigation
- Mesh visualization settings

## Nodes

| Node | Description |
|------|-------------|
| `road_mask_extractor` | Extracts blue road regions from color mask |
| `depth_masker` | Applies road mask to depth image |
| `camera_info_publisher` | Generates CameraInfo for masked depth |

## Saving the 3D Map

```bash
# Save as PLY file
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '/tmp/road_map.ply'}"

# Save full map (for later loading)
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath "{file_path: '/tmp/road_map.nvblx'}"
```

## Visualization in RViz2

1. Add `Image` display for `/segmentation/road_mask`
2. Add `Image` display for `/nvblox/depth/image`
3. Add nvblox mesh plugin for 3D reconstruction visualization

## License

MIT