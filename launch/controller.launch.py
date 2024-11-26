from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller',
            namespace='controller',
            executable='controller',
            name='controller'
        ),
        Node(
            package='motor_controller',
            namespace='estop',
            executable='estop',
            name='estop'
        ),
        Node(
            package='motor_controller',
            namespace='relay',
            executable='relay',
            name='relay'
        ),
        Node(
            package='motor_controller',
            namespace='salter',
            executable='salter',
            name='salter'
        ),
    ])