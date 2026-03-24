# aruco_depth_fusion

ROS 2 Python package for ArUco pose estimation using RGB-D fusion in simulation and real sensor pipelines.

This node is part of the CHARS execution and perception layer. It detects ArUco markers from RGB images, fuses depth at marker center pixels, and publishes precise TF frames for downstream pick-and-place actions.

## What This Package Does

- Detects ArUco markers (`DICT_6X6_250`) from RGB frames.
- Synchronizes RGB and depth streams with `ApproximateTimeSynchronizer`.
- Computes 3D translation using camera intrinsics + depth value.
- Uses OpenCV pose estimation for marker orientation.
- Publishes marker transforms to `/tf` as `aruco_box_<id>` frames.
- Falls back to OpenCV translation when depth is invalid.

## CHARS Context

In CHARS, this package provides high-precision perception for Layer 1 (Execution and Perception):

- Robots navigate/manipulate via Nav2 + MoveIt2.
- `aruco_depth_fusion` continuously updates object poses in TF.
- Local action servers query TF at execution time to improve pick/place accuracy.

## Package Structure

```text
aruco_depth_fusion/
  aruco_depth_fusion/
    aruco_fusion_node.py
  launch/
    aruco_fusion.launch.py
  package.xml
  setup.py
```

## Dependencies

Declared in `package.xml`:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `message_filters`
- `tf2_ros`
- `python3-opencv`
- `python3-numpy`
- `python3-scipy`

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select aruco_depth_fusion
source install/setup.bash
```

## Run

### 1. Run node directly

```bash
ros2 run aruco_depth_fusion aruco_fusion_node
```

### 2. Run with launch file

```bash
ros2 launch aruco_depth_fusion aruco_fusion.launch.py
```

## Default Topics and Frames

### Topics subscribed by the node

- RGB image: `/cam1/depth_camera/image`
- Depth image: `/cam1/depth_camera/depth_image`
- Camera info: `/cam1/depth_camera/camera_info`

### TF output

- Parent frame: `cam1/optical_frame`
- Child frame(s): `aruco_box_<marker_id>`

### Visualization

The node opens an OpenCV window:

- Marker boundaries and center points
- Fused axis projection
- Pose label with source (`DEPTH` or `OPENCV`)

## Verify Output

```bash
# Check TF stream
ros2 topic echo /tf

# List available frames
ros2 run tf2_tools view_frames
```

## Important Note About Launch Arguments

The launch file declares configurable arguments (`rgb_topic`, `depth_topic`, `camera_info_topic`, `marker_size`, etc.).

Current node implementation uses hardcoded topic names and marker size internally. If launch arguments are changed, the node will not yet apply them until parameter handling is added in `aruco_fusion_node.py`.

## Typical Use in Gazebo Fortress

1. Start simulation and spawn camera + ArUco targets.
2. Start this node.
3. Confirm `aruco_box_*` frames are appearing in `/tf`.
4. Use TF lookups in robot action servers for precise end-effector alignment.

## Troubleshooting

- No detections:
  - Confirm marker dictionary is `DICT_6X6_250`.
  - Verify camera image contains visible markers.
- Invalid depth warnings:
  - Check depth stream encoding and range.
  - Ensure marker center pixels are inside valid depth regions.
- No TF frames:
  - Confirm `camera_info` is published and intrinsics are valid.
  - Verify synchronized RGB/depth streams are active.

## License

Apache-2.0
