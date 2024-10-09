import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    xacro_file = 'urdfbot_simple.xacro'
    package_description = "urdfbot_description"
    xacro_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    robot_description = Command(['xacro ', xacro_path])
        
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
        output="screen"
    )

    odometry_node = Node(
        package='Odometry_package',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # wheel_to_odometry_node = Node(
    #     package='Odometry_package',
    #     executable='wheel_to_odometry_node',
    #     name='wheel_to_odometry_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )

    #  cmd_vel_to_wheel_node = Node(
    #     package='Odometry_package',
    #     executable='cmd_vel_to_wheel_node',
    #     name='cmd_vel_to_wheel_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )

    # odom_base_link_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     namespace='',
    #     output='screen',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']
    # )
    
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui_',
    #     executable='joint_state_publisher_gui_',
    #     name='joint_state_publisher_gui_',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )

    # keyboard_control_node = Node(
    #     package='Odometry_package',
    #     executable='keyboard_control_node',
    #     name='keyboard_control_node',
    #     output='screen'
    # )    

    return LaunchDescription([
            DeclareLaunchArgument(
                'urdf_file',
                default_value=xacro_path,
                description='Path to the URDF file'),

            # odom_base_link_node
            robot_state_publisher_node
            ,odometry_node
            # r,viz2_node
        ]
    )
