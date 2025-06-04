from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the audio node
        Node(
            package='tino_ros',
            executable='audio_node',
            name='audio_node',
            output='screen'
        ),
        
        # Launch the audio test node
        Node(
            package='tino_ros',
            executable='audio_test_node',
            name='audio_test_node',
            output='screen'
        ),

        # Launch the gamepad node
        Node(
            package='tino_ros',
            executable='gamepad_node',
            name='gamepad_node',
            output='screen'
        ),
    ])
