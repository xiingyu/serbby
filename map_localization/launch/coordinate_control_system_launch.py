from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_localization',  
            executable='coordinate_controler', 
            name='coordinate_controler',
            output='screen'
        ),
        Node(
            package='map_localization',  
            executable='current_coordinate', 
            name='current_coordinate',
            output='screen'
        )
    ])