#!/usr/bin/env python3
# filepath: /home/orinano/Tino-Robot/tino_ws/src/tino_ros/tino_ros/vr_data_recorder_node.py

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import os
import time
import datetime
import sqlite3
import json
from pathlib import Path

class MessageDataEntry:
    def __init__(self, topic, msg_type, timestamp, data):
        self.topic = topic
        self.msg_type = msg_type
        self.timestamp = timestamp
        self.data = data

class VRDataRecorderNode(Node):
    def __init__(self):
        super().__init__('vr_data_recorder_node')
        
        # Declare parameters
        self.declare_parameter(
            'recording_enabled', 
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Enable or disable recording')
        )
        self.declare_parameter(
            'data_directory', 
            str(Path.home() / 'vr_recordings'),
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Directory to save recordings')
        )
        self.declare_parameter(
            'max_recording_time_minutes', 
            30,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Maximum recording time in minutes')
        )
        
        # Set up recording database
        self.init_recording()
        
        # Subscribe to robot pose from controller
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/vr_in/robot_pose', 
            self.pose_callback,
            10)
            
        # Subscribe to human position data
        self.human_position_sub = self.create_subscription(
            PoseStamped,
            '/vr_in/human_position',
            self.human_position_callback,
            10)
            
        # Subscribe to human skeleton data
        self.human_skeleton_sub = self.create_subscription(
            MarkerArray,
            '/vr_in/human_skeleton',
            self.human_skeleton_callback,
            10)
            
        self.human_skeleton_poses_sub = self.create_subscription(
            PoseArray,
            '/vr_in/human_skeleton_poses',
            self.human_skeleton_poses_callback,
            10)
        
        # Subscribe to Audio from controller
        self.mic_audio_sub = self.create_subscription(
            Int16MultiArray, 
            '/vr_in/audio_output', 
            self.mic_audio_callback, 
            10)

        # Create a timer for periodic saves
        self.timer = self.create_timer(10.0, self.timer_callback)
        
        # Create services for control - use relative names
        # ROS2 will automatically prepend the node name to create the fully qualified service name
        self.start_service = self.create_service(
            Trigger, 'start', self.start_callback)
        self.stop_service = self.create_service(
            Trigger, 'stop', self.stop_callback)
        self.status_service = self.create_service(
            Trigger, 'status', self.status_callback)
        
        self.get_logger().info(f'VR Data Recorder node initialized, saving to {self.data_dir}')
        self.get_logger().info('Recording services available:')
        self.get_logger().info('  - Start: ros2 service call /vr_data_recorder_node/start std_srvs/srv/Trigger')
        self.get_logger().info('  - Stop: ros2 service call /vr_data_recorder_node/stop std_srvs/srv/Trigger')
        self.get_logger().info('  - Status: ros2 service call /vr_data_recorder_node/status std_srvs/srv/Trigger')

    def init_recording(self):
        """Initialize recording database and directories"""
        # Get parameters
        self.recording_enabled = self.get_parameter('recording_enabled').value
        self.data_dir = self.get_parameter('data_directory').value
        self.max_recording_time = self.get_parameter('max_recording_time_minutes').value * 60  # convert to seconds
        
        # Create directory if it doesn't exist
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Create a new recording session
        self.session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.data_dir, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        
        # Create database
        self.db_path = os.path.join(self.session_dir, 'recording.db')
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        
        # Create tables
        self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY,
            topic TEXT,
            msg_type TEXT,
            timestamp REAL,
            data BLOB
        )
        ''')
        self.conn.commit()
        
        # Recording stats
        self.start_time = time.time()
        self.message_counts = {
            '/vr_in/robot_pose': 0,
            '/vr_in/human_position': 0,
            '/vr_in/human_skeleton': 0,
            '/vr_in/human_skeleton_poses': 0,
            '/vr_in/audio_output': 0
        }
        
        # Create metadata file
        self.write_metadata()

    def write_metadata(self):
        """Write metadata about the recording session"""
        metadata = {
            'session_id': self.session_id,
            'start_time': datetime.datetime.fromtimestamp(self.start_time).isoformat(),
            'topics': list(self.message_counts.keys()),
            'data_format': 'SQLite database with serialized ROS messages',
            'max_recording_time_minutes': self.max_recording_time / 60
        }
        
        with open(os.path.join(self.session_dir, 'metadata.json'), 'w') as f:
            json.dump(metadata, f, indent=2)

    def update_metadata(self):
        """Update metadata with recording statistics"""
        elapsed_time = time.time() - self.start_time
        
        metadata = {
            'session_id': self.session_id,
            'start_time': datetime.datetime.fromtimestamp(self.start_time).isoformat(),
            'end_time': datetime.datetime.now().isoformat(),
            'duration_seconds': elapsed_time,
            'topics': list(self.message_counts.keys()),
            'message_counts': self.message_counts,
            'total_messages': sum(self.message_counts.values()),
            'data_format': 'SQLite database with serialized ROS messages',
            'db_path': self.db_path
        }
        
        with open(os.path.join(self.session_dir, 'metadata.json'), 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def save_message(self, topic, msg_type, msg):
        """Save a message to the database"""
        if not self.recording_enabled:
            return
            
        # Serialize message
        serialized_msg = serialize_message(msg)
        
        # Save to database
        timestamp = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        self.cursor.execute(
            "INSERT INTO messages (topic, msg_type, timestamp, data) VALUES (?, ?, ?, ?)",
            (topic, msg_type, timestamp, serialized_msg)
        )
        
        # Update message count
        self.message_counts[topic] += 1
        
        # Every 100 messages, commit to database
        if sum(self.message_counts.values()) % 100 == 0:
            self.conn.commit()
    
    def pose_callback(self, msg):
        """Record robot pose data"""
        self.save_message('/vr_in/robot_pose', 'PoseWithCovarianceStamped', msg)
    
    def human_position_callback(self, msg):
        """Record human position data"""
        self.save_message('/vr_in/human_position', 'PoseStamped', msg)
    
    def human_skeleton_callback(self, msg):
        """Record human skeleton visualization data"""
        self.save_message('/vr_in/human_skeleton', 'MarkerArray', msg)
    
    def human_skeleton_poses_callback(self, msg):
        """Record human skeleton poses data"""
        self.save_message('/vr_in/human_skeleton_poses', 'PoseArray', msg)
    
    def mic_audio_callback(self, msg):
        """Record audio data from microphone"""
        self.save_message('/vr_in/audio_output', 'Int16MultiArray', msg)
    
    def timer_callback(self):
        """Periodic timer to commit data and check recording status"""
        if not self.recording_enabled:
            return
            
        # Commit any pending data
        self.conn.commit()
        
        # Check if we've exceeded the max recording time
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.max_recording_time:
            self.get_logger().info(f'Maximum recording time reached ({self.max_recording_time/60:.1f} minutes)')
            self.stop_recording()
            return
            
        # Log current status
        total_msgs = sum(self.message_counts.values())
        self.get_logger().info(f'Recording in progress - {elapsed_time:.1f}s elapsed, {total_msgs} messages recorded')
        for topic, count in self.message_counts.items():
            if count > 0:
                self.get_logger().debug(f'  - {topic}: {count} messages')
    
    def start_callback(self, request, response):
        """Handle start recording service call"""
        if not self.recording_enabled:
            self.start_recording()
            response.success = True
            response.message = "Recording started"
        else:
            response.success = False
            response.message = "Recording is already active"
        return response
    
    def stop_callback(self, request, response):
        """Handle stop recording service call"""
        if self.recording_enabled:
            self.stop_recording()
            response.success = True
            response.message = "Recording stopped"
        else:
            response.success = False
            response.message = "Recording is already stopped"
        return response
    
    def status_callback(self, request, response):
        """Handle status check service call"""
        if self.recording_enabled:
            elapsed_time = time.time() - self.start_time
            total_msgs = sum(self.message_counts.values())
            response.success = True
            response.message = f"Recording active - {elapsed_time:.1f}s elapsed, {total_msgs} messages"
        else:
            response.success = True
            response.message = "Recording is stopped"
        return response
    
    def start_recording(self):
        """Start a new recording session"""
        self.get_logger().info("Starting new recording session")
        
        # Close previous session if any
        if hasattr(self, 'conn') and self.conn:
            self.conn.close()
        
        # Initialize new recording
        self.init_recording()
        self.recording_enabled = True
    
    def stop_recording(self):
        """Stop the current recording session"""
        if not self.recording_enabled:
            return
            
        self.recording_enabled = False
        
        # Commit any pending changes
        self.conn.commit()
        
        # Update metadata
        self.update_metadata()
        
        # Close connection
        self.conn.close()
        
        elapsed_time = time.time() - self.start_time
        total_msgs = sum(self.message_counts.values())
        self.get_logger().info(f'Recording stopped - {elapsed_time:.1f}s elapsed, {total_msgs} total messages')
        self.get_logger().info(f'Data saved to {self.session_dir}')
        
        # Log message counts
        for topic, count in self.message_counts.items():
            self.get_logger().info(f'  - {topic}: {count} messages')

def main(args=None):
    rclpy.init(args=args)
    recorder_node = VRDataRecorderNode()
    
    # No custom signal handlers - let rclpy handle SIGINT
    try:
        recorder_node.get_logger().info('VR Data Recorder running. Press Ctrl+C to stop.')
        rclpy.spin(recorder_node)
    except KeyboardInterrupt:
        recorder_node.get_logger().info('Keyboard interrupt detected, stopping recording...')
        if recorder_node.recording_enabled:
            recorder_node.stop_recording()
    except Exception as e:
        recorder_node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        recorder_node.get_logger().info('Shutting down...')
        recorder_node.destroy_node()
        rclpy.shutdown()
        recorder_node.get_logger().info('Shutdown complete.')
        # Force exit to ensure we don't get stuck
        import sys
        sys.exit(0)


if __name__ == '__main__':
    main()
