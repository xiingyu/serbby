from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',  
            executable='ramp_controll',  
            output='screen'  
        ),
        Node(
            package='joy',  
            executable='joy_node',  
            output='screen'  
        ),
        Node(
            package='controller',  
            executable='complete_joy_drive',  
            output='screen'  
        ),
        
    ])
