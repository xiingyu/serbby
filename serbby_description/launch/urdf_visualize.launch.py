import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


import os.path

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'serbby_description.urdf'
    rviz_config_file = "serbby_examples.rviz"
    # xacro_file = "urdfbot.xacro"
    package_description = "serbby_description"
    ####### DATA INPUT END ##########

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    rviz_config_path =os.path.join(get_package_share_directory(package_description), "rviz", rviz_config_file)

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )
    
    open_rviz2_with_config_file = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + rviz_config_path]
        )
    
    run_joint_publisher = Node(
            package='joint_state_publisher_gui',
            namespace='',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': True}],
            output="screen"
    )
    
    
    # tf_map_map_base = Node(
    #         package='tf2_ros',
    #         namespace='',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0 0 0 0 0 0 map base_link']
    # )
    

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node,
            open_rviz2_with_config_file,
            
            
            # tf_map_map_base,
            run_joint_publisher,
        ]
    )