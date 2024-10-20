from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Odometry_package',
            executable='cmd_vel_to_wheel_node',
            name='cmd_vel_to_wheel_node',
            output='screen'
        ),
        Node(
            package='Odometry_package',
            executable='wheel_to_odometry_node',
            name='wheel_to_odometry_node',
            output='screen'
        ),
        Node(
            package='Odometry_package',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        )
    ])