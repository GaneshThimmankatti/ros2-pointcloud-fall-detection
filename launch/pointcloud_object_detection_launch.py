from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import numpy as np


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_detection',
            executable='pointcloud_transformer',
            name='PointcloudTransformer',
            output='screen',
            parameters=[{
                'sub_topic_name': '/camera/camera/depth/color/points',
                'pub_topic_name': '/camera/camera/depth/color/points_transformed',
                'target_frame': 'base_link',
                'subsampling_ratio': 10
            }],

        ),

        Node(
            package='pointcloud_detection',
            executable='pointcloud_object_detection',
            name='PointcloudObjectDetection',
            output='screen',
            parameters=[{
                'fov_angle_deg': 35.0,
                'ground_distance_min': 2.0,
                'ground_distance_max': 3.2,
                'ground_plane_range': 0.2,
                'threshold': 10,
                'max_object_height': 1.8,
                'height_over_ground': 0.15,
                'sub_topic_name': '/camera/camera/depth/color/points_transformed',
                'pub_topic_name': '/camera/camera/depth/color/scan',
                'target_frame': 'base_link',
                'range_max': 4.0,
            }],
        )
    ])
