from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for base Arduino'
        ),
        DeclareLaunchArgument(
            'leg_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for leg Arduino'
        ),
        DeclareLaunchArgument(
            'head_port',
            default_value='/dev/ttyUSB2',
            description='Serial port for head Arduino'
        ),

        # Start Robot Controller Node
        Node(
            package='tino_ros',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),
        
        # Start the gamepad node
        Node(
            package='tino_ros',
            executable='gamepad_node',
            name='gamepad_node',
            output='screen'
        ),

        # Start the VR interface node
        Node(
            package='tino_ros',
            executable='vr_interface_node',
            name='vr_interface_node',
            output='screen'
        ),
        
        # Start the hardware interface node
        Node(
            package='tino_ros',
            executable='hardware_interface',
            name='hardware_interface',
            parameters=[{
                'base_port': LaunchConfiguration('base_port'),
                'leg_port': LaunchConfiguration('leg_port'),
                'head_port': LaunchConfiguration('head_port'),
                'baud_rate': 115200,
                'msg_repeat': 3
            }],
            output='screen'
        ),
        
    ])