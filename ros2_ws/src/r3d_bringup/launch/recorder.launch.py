import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_output = os.path.join(
        os.path.expanduser('~'), 'Desktop', 'save_pcd', 'auto'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('odom_topic', default_value='/lio_sam/mapping/odometry'),
            DeclareLaunchArgument('output_dir', default_value=default_output),
            DeclareLaunchArgument('camera_device0', default_value='/dev/video0'),
            DeclareLaunchArgument('camera_device2', default_value='/dev/video2'),
            DeclareLaunchArgument('image_width', default_value='2592'),
            DeclareLaunchArgument('image_height', default_value='1944'),
            DeclareLaunchArgument('fps', default_value='10'),
            DeclareLaunchArgument('jpeg_quality', default_value='90'),
            DeclareLaunchArgument('max_pending', default_value='16'),
            DeclareLaunchArgument('record_bag', default_value='true'),
            DeclareLaunchArgument(
                'bag_topics', default_value='/livox/imu /livox/lidar'
            ),
            Node(
                package='r3d_bringup',
                executable='image_odom_recorder',
                name='image_odom_recorder',
                output='screen',
                parameters=[
                    {
                        'odom_topic': LaunchConfiguration('odom_topic'),
                        'output_dir': LaunchConfiguration('output_dir'),
                        'camera_device0': LaunchConfiguration('camera_device0'),
                        'camera_device2': LaunchConfiguration('camera_device2'),
                        'image_width': LaunchConfiguration('image_width'),
                        'image_height': LaunchConfiguration('image_height'),
                        'fps': LaunchConfiguration('fps'),
                        'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                        'max_pending': LaunchConfiguration('max_pending'),
                        'record_bag': LaunchConfiguration('record_bag'),
                        'bag_topics': LaunchConfiguration('bag_topics'),
                    }
                ],
            ),
        ]
    )
