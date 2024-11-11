import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    odometry = get_package_share_directory('Odometry_package')
    local = get_package_share_directory('map_localization')

    ydlidar_dir = os.path.join(ydlidar, 'launch')
    odometry_dir = os.path.join(odometry, 'launch')
    local_dir = os.path.join(local, 'launch')


    ydlidar_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_dir, '/nav_launch.py']),
        launch_arguments={'arg_name': 'arg_value'}.items()  
    )

    ydlidar_rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_dir, '/rviz2_launch.py']),
        launch_arguments={'arg_name': 'arg_value'}.items()  
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([odometry_dir, '/odometry_launch.py']),
        launch_arguments={'arg_name': 'arg_value'}.items()  
    )

    local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([local_dir, '/coordinate_control_system_launch.py']),
        launch_arguments={'arg_name': 'arg_value'}.items()  
    )

    return LaunchDescription([
        ydlidar_nav2_launch,
        ydlidar_rviz2_launch,
        odometry_launch,
        local_launch
    ])
