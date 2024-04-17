from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_pkg', executable='simpletimepub', output = 'screen'),
        Node(package='my_pkg', executable='simpletimesub', output = 'screen')
        ])
