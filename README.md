# Lane Mapping ACC

This package provides road segmentation and masking for 3D mapping using Isaac ROS Nvblox and RTAB-Map.

## Quick Start: Launch Order

To run the full system, execute these launch files in order:

1.  **Robot Drivers**: Start your robot hardware usually with `ros2 launch qcar2_bringup bringup.launch.py` (Camera + IMU).
2.  **Segmentation**: `ros2 launch lane_mapping_acc road_segmentation.launch.py`
3.  **Mapping**: `ros2 launch lane_mapping_acc rtabmap_mapping.launch.py`

## RTAB-Map Integration

To run RTAB-Map with visual odometry and 3D mapping using masked depth images:

```bash
ros2 launch lane_mapping_acc rtabmap_mapping.launch.py
```

### Configuration

- **Visual Odometry**: Uses raw depth from `/camera/depth_image` and RGB from `/camera/color_image`.
- **Mapping (SLAM)**: Uses masked depth from `/nvblox/depth/image` (road only) and IMU from `/qcar2_imu`.

### Static Transform

The launch file includes the following static transform for the camera:

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.095 --y 0.032 --z 0.172 \
    --roll -1.5708 --pitch 0 --yaw -1.5708 \
    --frame-id base_link \
    --child-frame-id camera_depth_optical_frame
```

## Nodes

### Depth Masker
- Input: `/camera/depth_image`, `/segmentation/road_mask`
- Output: `/nvblox/depth/image` (masked depth)

### Camera Info Publisher
- Publishes synthesized `CameraInfo` for the masked depth image.


```bash

sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
sudo ln -sf message_filters/subscriber.h subscriber.hpp
sudo ln -sf message_filters/time_synchronizer.h time_synchronizer.hpp
sudo ln -sf message_filters/synchronizer.h synchronizer.hpp
sudo ln -sf message_filters/sync_policies sync_policies
sudo ln -sf approximate_time.h approximate_time.hpp
sudo ln -sf exact_time.h exact_time.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/Transform.h /opt/ros/humble/include/tf2/tf2/LinearMath/Transform.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/Vector3.h /opt/ros/humble/include/tf2/tf2/LinearMath/Vector3.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/Quaternion.h /opt/ros/humble/include/tf2/tf2/LinearMath/Quaternion.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.h /opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/Scalar.h /opt/ros/humble/include/tf2/tf2/LinearMath/Scalar.hpp
sudo ln -sf /opt/ros/humble/include/tf2/tf2/LinearMath/MinMax.h /opt/ros/humble/include/tf2/tf2/LinearMath/MinMax.hpp
sudo ln -sf tf2/buffer_core.h buffer_core.hpp
sudo ln -sf tf2/convert.h convert.hpp
sudo ln -sf tf2/exceptions.h exceptions.hpp
sudo ln -sf tf2/impl/utils.h utils.hpp
sudo ln -sf tf2/transform_datatypes.h transform_datatypes.hpp
sed -i 's/tf2::getYaw/tf2::impl::getYaw/g' /workspaces/isaac_ros-dev/ros2/src/rtabmap_ros/rtabmap_util/src/nodelets/imu_to_tf.cpp
```