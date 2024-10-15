import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import lifecycle_msgs.msg
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    # rviz_config_file = os.path.join(share_dir, 'config','ydlidar_cartographer.rviz')
    parameter_file = LaunchConfiguration('parameter_file',default=os.path.join(share_dir,'params','ydlidar.yaml'))
    # cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(share_dir, 'config'))
    # configuration_basename = LaunchConfiguration('configuration_basename', default='ydlidar_cartographer.lua')
    # resolution = LaunchConfiguration('resolution', default='0.05')
    # publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')


    driver_node=LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    
    params_declare = DeclareLaunchArgument('parameter_file',
                                           default_value=parameter_file,
                                           description='FPath to the ROS2 parameters file to use.')
    
    return LaunchDescription([
        # Node(
        #     ## Configure the TF of the robot to the origin of the map coordinates
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'laser_frame']
        #     ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0','0.0', '0.0', '0.0','base_link','laser_frame'],
            ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # map TF to odom TF
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # odom TF to base_link
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # odom TF to base_footprint
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
        ),

        DeclareLaunchArgument(
            'parameter_file',
            default_value=parameter_file,
            description='FPath to the ROS2 parameters file to use.'),

        driver_node,
        params_declare,
    ])
