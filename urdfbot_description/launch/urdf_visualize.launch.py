import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = 'urdfbot_simple.xacro'
    package_description = "urdfbot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
   
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )
    
    # join_state_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': True}],
    #     output="screen"
    # )

    # rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            # join_state_gui,
            rviz2
        ]
    )
