from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cobrotic',
            executable='moveit2_to_arduino',
            name='joint_state_to_arduino',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'topic': '/joint_states'}
            ]
        ),
    ])