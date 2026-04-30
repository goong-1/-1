import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('tracking_system')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='tracking_system',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tracking_system',
            executable='aruco_tf_node',
            name='aruco_tf_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tracking_system',
            executable='follow_target_tf_node',
            name='follow_target_tf_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tracking_system',
            executable='follower_controller_node',
            name='follower_controller_node',
            output='screen',
            parameters=[params_file],
        ),
    
        Node(
            package='tracking_system',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            parameters=[params_file],
        ),
    ])