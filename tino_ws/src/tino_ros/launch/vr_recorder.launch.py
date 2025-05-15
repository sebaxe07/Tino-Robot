#!/usr/bin/env python3
# filepath: /home/orinano/Tino-Robot/tino_ws/src/tino_ros/launch/vr_recorder.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    recording_enabled_arg = DeclareLaunchArgument(
        'recording_enabled',
        default_value='True',
        description='Enable or disable recording'
    )
    
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=os.path.expanduser('~/vr_recordings'),
        description='Directory to save recordings'
    )
    
    max_recording_time_arg = DeclareLaunchArgument(
        'max_recording_time',
        default_value='30',
        description='Maximum recording time in minutes'
    )
    
    # Create VR data recorder node
    vr_recorder_node = Node(
        package='tino_ros',
        executable='vr_data_recorder_node',
        name='vr_data_recorder_node',
        output='screen',
        parameters=[{
            'recording_enabled': LaunchConfiguration('recording_enabled'),
            'data_directory': LaunchConfiguration('data_dir'),
            'max_recording_time_minutes': LaunchConfiguration('max_recording_time'),
        }],
    )
    
    # Print instructions when launching
    from launch.actions import LogInfo
    recorder_instructions = LogInfo(msg="\n" + 
        "VR Data Recorder Node started. Available services:\n" +
        "===============================================\n" +
        "- Start recording: ros2 service call /vr_data_recorder_node/start std_srvs/srv/Trigger\n" +
        "- Stop recording:  ros2 service call /vr_data_recorder_node/stop std_srvs/srv/Trigger\n" +
        "- Check status:    ros2 service call /vr_data_recorder_node/status std_srvs/srv/Trigger\n\n" +
        "To extract recorded data:\n" +
        "- List recordings: ros2 run tino_ros vr_data_extractor --list\n" +
        "- Extract data:    ros2 run tino_ros vr_data_extractor --extract <session_path>\n" +
        "==============================================="
    )
    
    return LaunchDescription([
        recording_enabled_arg,
        data_dir_arg,
        max_recording_time_arg,
        vr_recorder_node,
        recorder_instructions
    ])
