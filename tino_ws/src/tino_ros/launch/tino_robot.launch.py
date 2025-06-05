from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessIO
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the tino_ros package directory
    tino_ros_dir = get_package_share_directory('tino_ros')

    # Define the TCP endpoint node so we can reference it in the event handler
    tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='tcp_endpoint',
        parameters=[{
            'ROS_IP': '192.168.0.105'
        }],
        output='screen',
    )
    
    # Register event handler to restart the TCP endpoint if it crashes (non-zero exit code)
    tcp_restart_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=tcp_endpoint_node,
            on_exit=lambda event, context: [
                # Check if exit code is non-zero (crash)
                Node(
                    package='ros_tcp_endpoint',
                    executable='default_server_endpoint',
                    name='tcp_endpoint',
                    parameters=[{
                        'ROS_IP': '192.168.0.105'
                    }],
                    output='screen',
                ) if event.returncode != 0 else None,
                # Log message about restart if crashed
                print(f"TCP Endpoint exited with code {event.returncode}. {'Restarting...' if event.returncode != 0 else 'Normal shutdown.'}")
            ]
        )
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/ttyBASE',  # Updated to new port
            description='Serial port for base Arduino'
        ),
        DeclareLaunchArgument(
            'leg_port',
            default_value='/dev/ttyLEG',  # Set to 'none' since it's not connected
            description='Serial port for leg Arduino'
        ),
        DeclareLaunchArgument(
            'head_port',
            default_value='/dev/ttyHEAD',  # Set to 'none' since it's not connected
            description='Serial port for head Arduino'
        ),
        
        # Start pose detection system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(tino_ros_dir, 'launch', 'pose_detection.launch.py')
            ]),
            # You can pass additional parameters if needed
            # launch_arguments={
            #    'confidence_threshold': '0.6',
            # }.items(),
        ),

        # Start Robot Controller Node
        Node(
            package='tino_ros',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),
        
        # # Start the gamepad node
        # Node(
        #     package='tino_ros',
        #     executable='gamepad_node',
        #     name='gamepad_node',
        #     output='screen'
        # ),

        # Start the VR interface node
        Node(
            package='tino_ros',
            executable='vr_interface_node',
            name='vr_interface_node',
            output='screen'
        ),

        # Start the Audio node
        Node(
            package='tino_ros',
            executable='audio_node',
            name='audio_node',
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
        
        # Use the TCP endpoint node defined above
        tcp_endpoint_node,
        
        # Add the restart handler
        tcp_restart_handler,
        
    ])
