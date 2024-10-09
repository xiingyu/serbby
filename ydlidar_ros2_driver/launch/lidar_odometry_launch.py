import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='ydlidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    return LaunchDescription([
            
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[('odom','rs_t265/odom'),]
            ),
            
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'
            ),

        # DeclareLaunchArgument(
        #     'publish_period_sec',
        #     default_value=publish_period_sec,
        #     description='OccupancyGrid publishing period'
        #     ),

        # Node(
        #     package='cartographer_ros',
        #     executable='cartographer_occupancy_grid_node',
        #     name='occupancy_grid_node',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
        #     )
    ])