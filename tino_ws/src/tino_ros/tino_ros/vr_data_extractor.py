#!/usr/bin/env python3
# filepath: /home/orinano/Tino-Robot/tino_ws/src/tino_ros/tino_ros/vr_data_extractor.py

import os
import sys
import json
import sqlite3
import argparse
import datetime
import numpy as np
from pathlib import Path

# Import ROS message types for deserialization
import rclpy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import MarkerArray
from rosidl_runtime_py.utilities import get_message

def list_recordings(base_dir):
    """List all available recording sessions"""
    base_path = Path(base_dir)
    
    if not base_path.exists() or not base_path.is_dir():
        print(f"Directory {base_path} does not exist!")
        return []
    
    sessions = []
    
    for session_dir in base_path.iterdir():
        if not session_dir.is_dir():
            continue
            
        metadata_file = session_dir / 'metadata.json'
        if metadata_file.exists():
            try:
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
                
                sessions.append({
                    'id': metadata.get('session_id', session_dir.name),
                    'path': str(session_dir),
                    'start_time': metadata.get('start_time', 'Unknown'),
                    'duration_seconds': metadata.get('duration_seconds', 'Unknown'),
                    'total_messages': metadata.get('total_messages', 'Unknown'),
                    'message_counts': metadata.get('message_counts', {})
                })
            except Exception as e:
                print(f"Error reading metadata from {metadata_file}: {e}")
    
    return sorted(sessions, key=lambda x: x['id'], reverse=True)

def extract_unity_format(session_path, output_dir=None, topic_filter=None):
    """Extract data from a recording session in a Unity-friendly format"""
    session_path = Path(session_path)
    
    if not session_path.exists() or not session_path.is_dir():
        print(f"Session directory {session_path} does not exist!")
        return False
    
    # Load metadata
    metadata_file = session_path / 'metadata.json'
    if not metadata_file.exists():
        print(f"Metadata file not found in {session_path}!")
        return False
        
    with open(metadata_file, 'r') as f:
        metadata = json.load(f)
    
    # Determine output directory
    if output_dir is None:
        output_dir = session_path / 'unity_format'
    else:
        output_dir = Path(output_dir)
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Connect to database
    db_path = session_path / 'recording.db'
    if not db_path.exists():
        print(f"Database file {db_path} not found!")
        return False
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get all topics
    cursor.execute("SELECT DISTINCT topic FROM messages")
    available_topics = [row[0] for row in cursor.fetchall()]
    
    # Apply topic filter if provided
    if topic_filter:
        topics_to_process = [topic for topic in available_topics if any(filter_topic in topic for filter_topic in topic_filter)]
    else:
        topics_to_process = available_topics
    
    print(f"Extracting data for topics: {topics_to_process}")
    
    # Process each topic
    for topic in topics_to_process:
        topic_safe_name = topic.replace('/', '_').lstrip('_')
        
        # Extract data for specific topic
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic = ? ORDER BY timestamp",
            (topic,)
        )
        
        rows = cursor.fetchall()
        print(f"Topic {topic}: {len(rows)} messages")
        
        if not rows:
            continue
        
        # For Unity integration, we'll create CSV or JSON files that Unity can easily parse
        # Here we create simplified JSON files containing the message data
        output_file = output_dir / f"{topic_safe_name}.json"
        
        # For each message type, we need to deserialize the ROS message
        # and extract the relevant fields
        
        # Create message type mapping
        msg_type_map = {
            '/vr_in/robot_pose': PoseWithCovarianceStamped,
            '/vr_in/human_position': PoseStamped,
            '/vr_in/human_skeleton': MarkerArray,
            '/vr_in/human_skeleton_poses': PoseArray,
            '/vr_in/audio_output': Int16MultiArray
        }
        
        # Get the appropriate message type for deserialization
        msg_type = msg_type_map.get(topic)
        
        if not msg_type:
            print(f"Warning: No message type mapping for topic {topic}, skipping...")
            continue
            
        frames = []
        
        # Process each message by deserializing and extracting data
        for timestamp, data_blob in rows:
            if data_blob is None:
                print(f"Warning: Empty data blob for {topic} at {timestamp}, skipping...")
                continue
                
            try:
                # Deserialize the message from the binary blob
                msg = deserialize_message(data_blob, msg_type)
                
                if topic == '/vr_in/robot_pose':
                    # Extract position and orientation from PoseWithCovarianceStamped
                    pos = msg.pose.pose.position
                    ori = msg.pose.pose.orientation
                    frame = {
                        "timestamp": timestamp,
                        "frame_id": msg.header.frame_id,
                        "position": [pos.x, pos.y, pos.z],
                        "orientation": [ori.x, ori.y, ori.z, ori.w]
                    }
                    frames.append(frame)
        
                elif topic == '/vr_in/human_position':
                    # Extract position from PoseStamped
                    pos = msg.pose.position
                    ori = msg.pose.orientation
                    frame = {
                        "timestamp": timestamp,
                        "frame_id": msg.header.frame_id,
                        "position": [pos.x, pos.y, pos.z],
                        "orientation": [ori.x, ori.y, ori.z, ori.w]
                    }
                    frames.append(frame)
                
                elif topic == '/vr_in/human_skeleton':
                    # Extract markers from MarkerArray
                    markers_data = []
                    for marker in msg.markers:
                        marker_data = {
                            "id": marker.id,
                            "frame_id": marker.header.frame_id,
                            "ns": marker.ns,
                            "type": marker.type,
                            "position": [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                            "orientation": [marker.pose.orientation.x, marker.pose.orientation.y, 
                                          marker.pose.orientation.z, marker.pose.orientation.w],
                            "scale": [marker.scale.x, marker.scale.y, marker.scale.z],
                            "color": [marker.color.r, marker.color.g, marker.color.b, marker.color.a]
                        }
                        markers_data.append(marker_data)
                    
                    frame = {
                        "timestamp": timestamp,
                        "markers": markers_data
                    }
                    frames.append(frame)
                
                elif topic == '/vr_in/human_skeleton_poses':
                    # Extract poses from PoseArray
                    poses_data = []
                    for pose in msg.poses:
                        pose_data = {
                            "position": [pose.position.x, pose.position.y, pose.position.z],
                            "orientation": [pose.orientation.x, pose.orientation.y, 
                                          pose.orientation.z, pose.orientation.w]
                        }
                        poses_data.append(pose_data)
                    
                    frame = {
                        "timestamp": timestamp,
                        "frame_id": msg.header.frame_id,
                        "poses": poses_data
                    }
                    frames.append(frame)
                
                elif topic == '/vr_in/audio_output':
                    # Extract audio data from Int16MultiArray
                    # Convert to Python list for JSON serialization
                    audio_samples = np.array(msg.data).tolist()
                    
                    frame = {
                        "timestamp": timestamp,
                        "audio_samples": audio_samples,
                        "layout": {
                            "dim_length": [dim.size for dim in msg.layout.dim],
                            "dim_labels": [dim.label for dim in msg.layout.dim],
                            "data_offset": msg.layout.data_offset
                        }
                    }
                    frames.append(frame)
                
                else:
                    # Generic format for other message types
                    frame = {
                        "timestamp": timestamp,
                        "raw_size": len(data_blob)
                    }
                    frames.append(frame)
            except Exception as e:
                print(f"Error deserializing message for {topic} at {timestamp}: {e}")
        
        # Create the output data structure
        output_data = {
            "topic": topic,
            "type": msg_type.__name__,
            "frames": frames
        }
        
        # Write the JSON file
        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        print(f"Created {output_file}")
    
    # Create a README file with instructions
    readme_file = output_dir / "README.md"
    with open(readme_file, 'w') as f:
        f.write(f"""# VR Recording Data - Unity Format

This directory contains data extracted from a ROS2 recording session in a format designed for Unity integration.

## Recording Information
- **Session ID:** {metadata.get('session_id', 'Unknown')}
- **Recorded on:** {metadata.get('start_time', 'Unknown')}
- **Duration:** {metadata.get('duration_seconds', 'Unknown')} seconds
- **Total messages:** {metadata.get('total_messages', 'Unknown')}

## Message Counts by Topic
{chr(10).join([f"- **{topic}:** {count} messages" for topic, count in metadata.get('message_counts', {}).items()])}

## Usage in Unity
The JSON files in this directory contain the message data in a simplified format that can be imported into Unity.
Each file represents a single topic, and contains an array of "frames" with timestamps and message data.

**Note:** This is placeholder data generated from the recording - in a real implementation, you would need to deserialize 
the binary ROS messages into a Unity-friendly format.

## Using this Data
1. Import these JSON files into your Unity project
2. Use a JSON parser to load the data
3. Play back the messages in Unity based on the timestamps

## Generated on
{datetime.datetime.now().isoformat()}
""")
    
    print(f"\nExtraction complete. Output saved to {output_dir}")
    return True

def main():
    parser = argparse.ArgumentParser(description='Extract VR recording data for Unity')
    
    parser.add_argument('--list', action='store_true', help='List all available recording sessions')
    parser.add_argument('--extract', help='Extract data from a specific session path')
    parser.add_argument('--output', help='Output directory for extracted data')
    parser.add_argument('--topics', nargs='+', help='Filter topics to extract (substring match)')
    parser.add_argument('--data-dir', default=os.path.expanduser('~/vr_recordings'), 
                      help='Base directory for recordings (default: ~/vr_recordings)')
    
    args = parser.parse_args()
    
    if args.list:
        print("Available recording sessions:")
        sessions = list_recordings(args.data_dir)
        
        if not sessions:
            print("No recording sessions found.")
            return
        
        for i, session in enumerate(sessions):
            print(f"\n{i+1}. Session: {session['id']}")
            print(f"   Path: {session['path']}")
            print(f"   Start time: {session['start_time']}")
            print(f"   Duration: {session['duration_seconds']} seconds")
            print(f"   Total messages: {session['total_messages']}")
            if session['message_counts']:
                print("   Message counts:")
                for topic, count in session['message_counts'].items():
                    print(f"     - {topic}: {count}")
        
        print("\nTo extract data, run with --extract <session_path>")
    
    elif args.extract:
        success = extract_unity_format(args.extract, args.output, args.topics)
        if not success:
            sys.exit(1)
    
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
