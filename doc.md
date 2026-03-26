# `aruco_depth_fusion` — Technical Documentation

> **Package version:** 1.0.0  
> **Build type:** `ament_python`  
> **License:** Apache-2.0  
> **ROS 2 distro target:** Humble / Iron (tested in Gazebo Fortress)

---

## 1. Purpose

`aruco_depth_fusion` is a ROS 2 Python package that provides **high-precision, real-time 3D pose estimation** of ArUco fiducial markers by fusing data from a depth camera's RGB stream and its co-registered depth stream.

### Why depth fusion instead of pure OpenCV pose estimation?

Traditional `cv2.aruco.estimatePoseSingleMarkers` computes the 3D translation of a marker from a single RGB frame alone. This method is scale-dependent and can introduce Z-axis jitter, especially in simulation where the virtual camera may not perfectly model lens distortion. 

The fusion approach fixes this by decoupling the two jobs of pose estimation:

| Responsibility | Source |
|---|---|
| **3D Translation (X, Y, Z)** | Depth sensor (accurate, physics-based) |
| **3D Orientation (Roll, Pitch, Yaw)** | OpenCV `solvePnP` on RGB corners (rotation cannot be inferred from a depth image alone) |

The result is a **hybrid pose** that is far more stable and accurate than either source alone.

### Role in CHARS

In the CHARS framework, this package is the perception backbone of **Layer 1 (Execution & Perception)**. The node runs continuously during a mission:

1. Gazebo spawns boxes with ArUco markers attached to them.
2. A depth camera mounted on or above the workspace observes these markers.
3. `aruco_depth_fusion` broadcasts each marker's pose as a TF frame (e.g., `aruco_box_3`).
4. When a robot reaches its Pick or Place waypoint, its **Action Server** performs a `tf_lookup` from the end-effector frame to the relevant `aruco_box_<id>` frame to get a precise, sub-centimetre offset for final arm alignment.

---

## 2. Package Structure

```
aruco_depth_fusion/
├── aruco_depth_fusion/
│   ├── __init__.py
│   └── aruco_fusion_node.py      ← single ROS 2 node implementation
├── launch/
│   └── aruco_fusion.launch.py    ← launch file with configurable arguments
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## 3. Dependencies

Declared in `package.xml`:

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client library |
| `sensor_msgs` | `Image`, `CameraInfo` message types |
| `geometry_msgs` | `TransformStamped` message type |
| `cv_bridge` | Converts ROS `Image` messages to OpenCV `numpy` arrays |
| `message_filters` | Time-synchronised multi-topic subscription |
| `tf2_ros` | TF2 transform broadcasting |
| `python3-opencv` | ArUco detection, pose estimation, visualization |
| `python3-numpy` | Numerical operations |
| `python3-scipy` | Rotation representation (`Rotation.from_rotvec`) |

---

## 4. Node: `aruco_depth_fusion_node`

### 4.1 Overview

| Property | Value |
|---|---|
| **Node name** | `aruco_depth_fusion_node` |
| **Class** | `ArucoDepthFusionNode` |
| **Executable** | `aruco_fusion_node` |
| **Entry point** | `aruco_depth_fusion.aruco_fusion_node:main` |

The node runs a single-threaded spin loop. All perception work happens inside two callbacks:

- `info_callback` — latches camera intrinsics on the first `CameraInfo` message.
- `synchronized_callback` — the main pipeline, fired once per time-synchronised RGB+depth pair.

---

### 4.2 Subscribed Topics

| Topic | Message Type | QoS | Description |
|---|---|---|---|
| `/cam1/depth_camera/image` | `sensor_msgs/Image` | RELIABLE, KEEP_LAST(10) | RGB (colour) image from the depth camera. Used for 2D ArUco detection and orientation estimation. |
| `/cam1/depth_camera/depth_image` | `sensor_msgs/Image` | RELIABLE, KEEP_LAST(10) | Depth image (`32FC1` encoding, values in metres). Used to sample the true Z distance at each marker centre pixel. |
| `/cam1/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE, KEEP_LAST(10) | Camera intrinsic matrix **K** and distortion coefficients **D**. Consumed once and latched. |

> **Note:** The RGB and depth topics are consumed through `message_filters.ApproximateTimeSynchronizer` with `queue_size=10` and `slop=0.1 s`. The `CameraInfo` topic is a standalone subscription that only processes the first received message.

---

### 4.3 Published Topics / TF Frames

This node does **not** publish to any standard ROS topic. Instead, it uses the **TF2 dynamic broadcaster** to publish transforms directly to the global `/tf` topic.

| TF Child Frame | Parent Frame | Description |
|---|---|---|
| `aruco_box_<marker_id>` | `cam1/optical_frame` | One transform per visible marker, where `<marker_id>` is the integer ArUco ID (e.g., `aruco_box_0`, `aruco_box_3`). Updated every synchronized RGB-D frame. The transform's **timestamp** is taken from the RGB message header so it is consistent with the image pipeline. |

Downstream consumers (e.g., robot action servers) should call `tf_buffer.lookup_transform('map', 'aruco_box_<id>', ...)` after chain-resolving through the camera → robot → map chain.

---

## 5. Algorithm Walkthrough

The `synchronized_callback` implements a four-step pipeline per detected marker:

### Step A — 2D Detection (RGB)

```python
corners, ids, rejected = cv2.aruco.detectMarkers(
    rgb_image, aruco_dict, parameters=aruco_params
)
```

- Dictionary: **`DICT_6X6_250`** (6×6 bit, up to 250 unique IDs).
- Returns pixel-space corner coordinates for each detected marker.
- Simultaneously, `cv2.aruco.estimatePoseSingleMarkers` is called to obtain rotation vectors (`rvecs`) for each marker — these are the orientations that the depth image cannot provide.

### Step B — Marker Centre Pixel Calculation

```
center_x = mean(corners[:, 0])   # average of 4 corner X-pixels
center_y = mean(corners[:, 1])   # average of 4 corner Y-pixels
```

The centre pixel `(pixel_u, pixel_v)` is where the depth sample is taken. Bounds checking is applied to guard against markers partially outside the image.

### Step C — Depth Sampling

```python
depth_value = depth_image[pixel_v, pixel_u]   # metres, float32
```

The raw depth value in metres is read directly from the `32FC1` depth image. If the value is `NaN`, `Inf`, or `≤ 0`, the node falls back to OpenCV's Z estimate for that marker.

### Step D — 3D Reconstruction (Pinhole Camera Model)

Using the camera intrinsics from `CameraInfo`:

```
Z = depth_value          (true metric depth from sensor)
X = (center_x - cx) * Z / fx
Y = (center_y - cy) * Z / fy
```

This is the standard **pinhole back-projection** formula. The result is the 3D position of the marker centre in the camera optical frame.

### Step E — Hybrid Pose Assembly and TF Broadcast

A `geometry_msgs/TransformStamped` is assembled:

- **Translation**: `(X, Y, Z)` from depth fusion (or OpenCV fallback).
- **Rotation**: quaternion converted from the `rvec` rotation vector via `scipy.spatial.transform.Rotation.from_rotvec(...).as_quat()`.
- **Header stamp**: copied from the RGB message timestamp.
- **Frame IDs**: `cam1/optical_frame` → `aruco_box_<id>`.

The transform is broadcast via `tf2_ros.TransformBroadcaster.sendTransform(...)`.

### Fallback Behaviour

| Condition | Behaviour |
|---|---|
| Depth value is valid | Full RGB-D fusion — labelled `DEPTH` in the overlay |
| Depth is `NaN` / `Inf` / `≤ 0` | Falls back to pure OpenCV tvec for translation — labelled `OPENCV` |
| `CameraInfo` not yet received | Warning logged, callback returns early |
| No markers detected | Image is still displayed; callback returns after the `cv2.imshow` call |

---

## 6. Parameters

> ⚠️ **Important:** The current node implementation uses **hardcoded** internal values. The launch file declares these as launch arguments and passes them as ROS 2 node parameters, but the node does not yet call `self.declare_parameter` / `self.get_parameter` internally. The table below describes both the hardcoded defaults in the node and the launch-file-declared defaults (which may differ).

| Parameter | Hardcoded Default (in node) | Launch-file Default | Type | Description |
|---|---|---|---|---|
| `rgb_topic` | `/cam1/depth_camera/image` | `/camera/image_raw` | `string` | RGB image topic to subscribe to |
| `depth_topic` | `/cam1/depth_camera/depth_image` | `/camera/depth_image` | `string` | Depth image topic to subscribe to |
| `camera_info_topic` | `/cam1/depth_camera/camera_info` | `/camera/camera_info` | `string` | Camera info topic |
| `marker_size` | `0.2` m (20 cm) | `0.05` m (5 cm) | `float` | Physical size of the ArUco marker (used only for OpenCV `estimatePoseSingleMarkers` — does not affect depth-fused X/Y/Z) |
| `sync_queue_size` | `10` | `10` | `int` | `ApproximateTimeSynchronizer` queue depth |
| `sync_slop` | `0.1` s | `0.1` s | `float` | Maximum timestamp difference (seconds) to consider two messages synchronised |
| `parent_frame` | `cam1/optical_frame` | *(not exposed)* | `string` | TF parent frame for all broadcast marker transforms |
| `aruco_dict` | `DICT_6X6_250` | *(not exposed)* | — | ArUco dictionary; hardcoded, not configurable via launch |

---

## 7. Building the Package

From the root of your ROS 2 workspace (e.g., `~/swarm_ws`):

```bash
# 1. Copy or symlink the package into your workspace src/
cp -r /path/to/aruco_depth_fusion ~/swarm_ws/src/

# 2. Build only this package
cd ~/swarm_ws
colcon build --packages-select aruco_depth_fusion

# 3. Source the install overlay
source install/setup.bash
```

---

## 8. Launching the Node

### Option A — Run directly (uses hardcoded topic names)

```bash
ros2 run aruco_depth_fusion aruco_fusion_node
```

### Option B — Launch file with default arguments

```bash
ros2 launch aruco_depth_fusion aruco_fusion.launch.py
```

> **Warning:** The launch-file default topics (`/camera/...`) differ from the node's hardcoded topics (`/cam1/depth_camera/...`). Until parameter handling is added to the node, use the `ros2 run` method when working with the Gazebo Fortress setup.

### Option C — Launch file with custom arguments

```bash
ros2 launch aruco_depth_fusion aruco_fusion.launch.py \
  rgb_topic:=/cam1/depth_camera/image \
  depth_topic:=/cam1/depth_camera/depth_image \
  camera_info_topic:=/cam1/depth_camera/camera_info \
  marker_size:=0.2 \
  sync_slop:=0.1
```

---

## 9. Verifying the Output

### Check TF is being broadcast

```bash
ros2 topic echo /tf
```

Look for transforms with `child_frame_id` values like `aruco_box_0`, `aruco_box_3`, etc.

### List all published TF frames

```bash
ros2 run tf2_tools view_frames
# Output saved to frames.pdf in the current directory
```

### Manual TF lookup

```bash
ros2 run tf2_ros tf2_echo cam1/optical_frame aruco_box_3
```

### Verify camera topics are active

```bash
ros2 topic list | grep cam1
ros2 topic hz /cam1/depth_camera/image
ros2 topic hz /cam1/depth_camera/depth_image
```

---

## 10. Visualization

The node opens an **OpenCV GUI window** titled `ArUco RGB-D Fusion` showing:

| Overlay | Description |
|---|---|
| Green polygon boundaries | ArUco marker bounding boxes |
| Magenta dot | Marker centre pixel (where depth is sampled) |
| Red line | Projected X-axis of the marker |
| Green line | Projected Y-axis of the marker |
| Yellow text label | `ID:<id> Z:<depth>m [DEPTH|OPENCV]` |

The window is resized to **960×540** for display. Press `Ctrl+C` in the terminal to shut down the node and close the window.

---

## 11. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| No markers detected, window shows blank image | ArUco dictionary mismatch or markers not in camera FOV | Ensure simulation markers use `DICT_6X6_250`; check camera pose in Gazebo |
| `Waiting for camera_info...` repeated | `CameraInfo` topic not publishing | Confirm camera plugin is active; check topic name with `ros2 topic list` |
| `Invalid depth` warnings for all markers | Depth image encoding mismatch or camera not publishing depth | Ensure depth topic uses `32FC1` encoding; check `ros2 topic info` |
| No TF frames appear | Messages not being time-synchronized | Increase `sync_slop` (e.g., to `0.5`); ensure both image topics are publishing at similar rates |
| TF frames jitter or jump | Noisy depth at marker centre pixel | Consider averaging depth over a small neighbourhood (code modification needed) |
| Launch file arguments have no effect | Node uses hardcoded values, not ROS parameters | Use `ros2 run` directly, or add `declare_parameter` / `get_parameter` calls to `aruco_fusion_node.py` |

---

## 12. Integration Notes for CHARS

- The node is designed to **always be running** during a CHARS mission, providing continuous perception even when no pick/place task is active.
- The child TF frames `aruco_box_<id>` are referenced by name in the Allocator's task definitions. Ensure the `marker_id` embedded in each task's JSON goal matches the physical marker IDs printed on the boxes.
- The TF chain used by action servers is typically:  
  `map → odom → base_link → ... → ee_link` and separately `cam1/optical_frame → aruco_box_<id>`.  
  The action server must look up the full chain `ee_link → aruco_box_<id>` (via the map anchor) to compute the delta pose for final arm alignment.
- Since the node broadcasts dynamic (live) TF frames, the action server should always use `rclpy.time.Time()` (latest available) for the `lookup_transform` call rather than a specific timestamp to avoid `ExtrapolationException`.
