from launch import LaunchDescription
from launch_ros.actions import node

def generate_launch_description():
    return LaunchDescription([
        node(
            package='demo_nodes_cpp',
            executable='talker'
        )
    ])