#!/usr/bin/env python3
"""
ArUco Depth Fusion Node - RGB-D Data Fusion

This node detects ArUco markers using RGB images and fuses with depth sensor
measurements for precise 3D pose estimation. Uses time-synchronized RGB-D
data to eliminate Z-jitter and scale dependency.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ArucoDepthFusionNode(Node):
    """Node to detect ArUco markers with RGB-D depth fusion."""

    def __init__(self):
        super().__init__('aruco_depth_fusion_node')
        
        # Topics for the depth camera (RGB + Depth from same sensor)
        self.rgb_topic = '/cam1/depth_camera/image'
        self.depth_topic = '/cam1/depth_camera/depth_image'
        self.camera_info_topic = '/cam1/depth_camera/camera_info'
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector (DICT_6X6_250 - matches your markers)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera intrinsics (will be populated from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info = None
        self.dist_coeffs = None
        self.camera_matrix = None
        
        # Marker size for rotation estimation (OpenCV still needs this)
        self.marker_size = 0.2  # 20cm markers
        
        # Message counters
        self.sync_count = 0
        
        # QoS profile - RELIABLE for Gazebo
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # TF broadcaster for publishing marker poses
        self.tf_broadcaster = TransformBroadcaster(self)
        self.parent_frame = 'cam1/optical_frame'  # Camera optical frame
        
        # Subscribe to CameraInfo first (standard subscriber)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, qos
        )
        
        # Time-synchronized subscribers using message_filters
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic, qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        
        # ApproximateTimeSynchronizer: queue_size=10, slop=0.1s
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # 100ms allowable delay for Gazebo simulation drift
        )
        self.time_sync.registerCallback(self.synchronized_callback)
        
        self.get_logger().info('=== ArUco RGB-D Fusion Node Started ===')
        self.get_logger().info(f'  RGB Topic: {self.rgb_topic}')
        self.get_logger().info(f'  Depth Topic: {self.depth_topic}')
        self.get_logger().info(f'  Parent Frame: {self.parent_frame}')
        self.get_logger().info(f'  ArUco Dictionary: DICT_6X6_250')
        self.get_logger().info(f'  Sync Policy: ApproximateTime (slop=0.1s)')
        self.get_logger().info('Waiting for synchronized RGB-D data...')

    def info_callback(self, msg: CameraInfo):
        """Callback for camera info - only process once."""
        if self.camera_info is None:
            self.camera_info = msg
            K = msg.k
            self.fx = K[0]
            self.fy = K[4]
            self.cx = K[2]
            self.cy = K[5]
            
            # Build camera matrix for OpenCV rotation estimation
            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ], dtype=np.float64)
            
            # Get distortion coefficients
            if len(msg.d) > 0:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            else:
                self.dist_coeffs = np.zeros(5, dtype=np.float64)
            
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def synchronized_callback(self, rgb_msg: Image, depth_msg: Image):
        """
        Synchronized callback for RGB and Depth images.
        Implements the RGB-D fusion algorithm.
        """
        self.sync_count += 1
        
        # Check if camera intrinsics are available
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera_info...')
            return
        
        # Convert messages to OpenCV images
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return
        
        # Make a copy for visualization
        display_img = rgb_image.copy()
        
        # ===== STEP A: 2D Detection (RGB) =====
        corners, ids, rejected = cv2.aruco.detectMarkers(
            rgb_image, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is None or len(ids) == 0:
            # No markers detected - still show the image
            display_resized = cv2.resize(display_img, (960, 540))
            cv2.imshow('ArUco RGB-D Fusion', display_resized)
            cv2.waitKey(1)
            return
        
        # Draw detected marker boundaries
        cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
        
        # Get rotation vectors from OpenCV (we still need this for orientation)
        rvecs, tvecs_opencv, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )
        
        # Process each detected marker
        markers_broadcast = []
        
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]  # Shape: (4, 2)
            
            # ===== NEW STEP: Calculate Center Pixel =====
            # Average of four corner coordinates
            center_x = np.mean(marker_corners[:, 0])
            center_y = np.mean(marker_corners[:, 1])
            
            # Convert to integer pixel coordinates
            pixel_u = int(round(center_x))
            pixel_v = int(round(center_y))
            
            # Ensure within image bounds
            h, w = depth_image.shape[:2]
            if not (0 <= pixel_u < w and 0 <= pixel_v < h):
                self.get_logger().warn(f'Marker {marker_id}: Center pixel out of bounds')
                continue
            
            # ===== STEP B: Depth Sampling =====
            depth_value = depth_image[pixel_v, pixel_u]
            
            # Validate depth value
            use_depth_fusion = True
            if np.isnan(depth_value) or np.isinf(depth_value) or depth_value <= 0.0:
                self.get_logger().warn(
                    f'Marker {marker_id}: Invalid depth ({depth_value}), '
                    f'falling back to OpenCV estimation'
                )
                use_depth_fusion = False
            
            # ===== STEP C & D: 3D Reconstruction / Hybrid Pose =====
            if use_depth_fusion:
                # Use depth sensor measurement (True Z)
                Z = float(depth_value)
                
                # Pinhole Camera Model: X = (u - cx) * Z / fx
                #                        Y = (v - cy) * Z / fy
                X = (center_x - self.cx) * Z / self.fx
                Y = (center_y - self.cy) * Z / self.fy
                
                pose_source = "DEPTH"
            else:
                # Fallback to OpenCV estimation
                X, Y, Z = tvecs_opencv[i][0]
                pose_source = "OPENCV"
            
            # Get rotation from OpenCV (depth sensor cannot detect rotation)
            rotation = R.from_rotvec(rvecs[i][0])
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            
            # ===== Broadcast TF Transform =====
            transform = TransformStamped()
            transform.header.stamp = rgb_msg.header.stamp  # Use RGB image timestamp
            transform.header.frame_id = self.parent_frame  # cam1/optical_frame
            transform.child_frame_id = f'aruco_box_{marker_id}'
            
            transform.transform.translation.x = X
            transform.transform.translation.y = Y
            transform.transform.translation.z = Z
            
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(transform)
            markers_broadcast.append(marker_id)
            
            # ===== Visualization =====
            # Draw center point
            cv2.circle(display_img, (pixel_u, pixel_v), 5, (255, 0, 255), -1)
            
            # Draw X/Y axes
            axis_length = self.marker_size * 0.5
            axis_points = np.float32([
                [0, 0, 0],
                [axis_length, 0, 0],
                [0, axis_length, 0],
            ]).reshape(-1, 3)
            
            # Use the fused translation for axis projection
            tvec_fused = np.array([[X, Y, Z]])
            img_points, _ = cv2.projectPoints(
                axis_points, rvecs[i], tvec_fused, self.camera_matrix, self.dist_coeffs
            )
            img_points = img_points.reshape(-1, 2).astype(int)
            
            origin = tuple(img_points[0])
            x_end = tuple(img_points[1])
            y_end = tuple(img_points[2])
            
            cv2.line(display_img, origin, x_end, (0, 0, 255), 3)  # X = Red
            cv2.line(display_img, origin, y_end, (0, 255, 0), 3)  # Y = Green
            
            # Add text label with pose info
            label = f'ID:{marker_id} Z:{Z:.2f}m [{pose_source}]'
            cv2.putText(display_img, label, (pixel_u - 50, pixel_v - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Log pose info
            self.get_logger().info(
                f'Marker {marker_id} [{pose_source}]: '
                f'X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m -> TF: aruco_box_{marker_id}'
            )
        
        # Log summary
        if markers_broadcast:
            self.get_logger().info(
                f'Sync #{self.sync_count}: Broadcasting {len(markers_broadcast)} '
                f'TF frames: {markers_broadcast}'
            )
        
        # Display result
        display_resized = cv2.resize(display_img, (960, 540))
        cv2.imshow('ArUco RGB-D Fusion', display_resized)
        cv2.waitKey(1)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ArucoDepthFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
