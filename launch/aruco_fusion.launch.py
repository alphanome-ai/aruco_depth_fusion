#!/usr/bin/env python3
"""
Launch file for ArUco Depth Fusion Node.

This launch file starts the aruco_fusion_node with configurable parameters
for topic names, marker size, and synchronization settings.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ArUco Depth Fusion node."""
    
    # Declare launch arguments
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/image_raw',
        description='RGB image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth_image',
        description='Depth image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera info topic'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='ArUco marker size in meters'
    )
    
    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='10',
        description='Message synchronization queue size'
    )
    
    sync_slop_arg = DeclareLaunchArgument(
        'sync_slop',
        default_value='0.1',
        description='Time synchronization slop in seconds'
    )
    
    # ArUco Depth Fusion Node
    aruco_fusion_node = Node(
        package='aruco_depth_fusion',
        executable='aruco_fusion_node',
        name='aruco_fusion_node',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'marker_size': LaunchConfiguration('marker_size'),
            'sync_queue_size': LaunchConfiguration('sync_queue_size'),
            'sync_slop': LaunchConfiguration('sync_slop'),
        }],
    )
    
    return LaunchDescription([
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        marker_size_arg,
        sync_queue_size_arg,
        sync_slop_arg,
        aruco_fusion_node,
    ])
