#!/usr/bin/env python3
"""
Test skeleton data generator for the VR interface node.
This simulates human skeleton poses for testing the VR interface.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import math
import time

class SkeletonTestNode(Node):
    def __init__(self):
        super().__init__('skeleton_test_node')
        
        # Publisher for skeleton poses
        self.skeleton_poses_pub = self.create_publisher(
            PoseArray,
            '/vr_in/human_skeleton_poses',
            10)
        
        # Timer to publish test data
        self.timer = self.create_timer(0.1, self.publish_test_skeleton)  # 10 Hz
        
        # Test parameters
        self.time_counter = 0.0
        self.joint_names = [
            'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
            'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
            'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
        ]
        
        self.get_logger().info('Skeleton test node initialized - publishing test skeleton data')
    
    def publish_test_skeleton(self):
        """Publish test skeleton data with animated movement"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'oak_right_camera_optical_frame'
        
        self.time_counter += 0.1
        
        # Create test poses for all 17 joints
        for i, joint_name in enumerate(self.joint_names):
            pose = Pose()
            
            # Create some animated test positions
            if 'eye' in joint_name or 'ear' in joint_name or joint_name == 'nose':
                # Head area - around (0, 0, 2.0)
                pose.position.x = 0.0 + 0.1 * math.sin(self.time_counter * 2)
                pose.position.y = -0.2 if 'left' in joint_name else 0.2
                pose.position.z = 2.0 + 0.05 * math.cos(self.time_counter * 2)
                
            elif 'shoulder' in joint_name:
                # Shoulders - wider
                pose.position.x = 0.0
                pose.position.y = -0.4 if 'left' in joint_name else 0.4
                pose.position.z = 1.8 + 0.1 * math.sin(self.time_counter)
                
            elif 'elbow' in joint_name:
                # Elbows - animated arm movement
                side = -1 if 'left' in joint_name else 1
                pose.position.x = 0.3 * math.sin(self.time_counter + side * math.pi/2)
                pose.position.y = side * (0.3 + 0.2 * math.cos(self.time_counter))
                pose.position.z = 1.5
                
            elif 'wrist' in joint_name:
                # Wrists - extend from elbows
                side = -1 if 'left' in joint_name else 1
                pose.position.x = 0.5 * math.sin(self.time_counter + side * math.pi/2)
                pose.position.y = side * (0.5 + 0.3 * math.cos(self.time_counter))
                pose.position.z = 1.4
                
            elif 'hip' in joint_name:
                # Hips
                pose.position.x = 0.0
                pose.position.y = -0.2 if 'left' in joint_name else 0.2
                pose.position.z = 1.2
                
            elif 'knee' in joint_name:
                # Knees - walking animation
                side = -1 if 'left' in joint_name else 1
                pose.position.x = 0.1 * math.sin(self.time_counter * 2 + side * math.pi)
                pose.position.y = side * 0.2
                pose.position.z = 0.8
                
            elif 'ankle' in joint_name:
                # Ankles - walking animation
                side = -1 if 'left' in joint_name else 1
                pose.position.x = 0.2 * math.sin(self.time_counter * 2 + side * math.pi)
                pose.position.y = side * 0.2
                pose.position.z = 0.1
            
            # Default orientation
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        # Simulate missing joints occasionally (every 5 seconds, remove some joints)
        if int(self.time_counter) % 5 == 0 and (self.time_counter % 1.0 < 0.2):
            # Remove some joints to test missing joint handling
            # Keep only first 12 joints (remove ankles, knees, some others)
            pose_array.poses = pose_array.poses[:12]
            self.get_logger().info(f'Simulating missing joints - only {len(pose_array.poses)}/17 joints')
        
        # Publish the skeleton data
        self.skeleton_poses_pub.publish(pose_array)
        
        # Log every 2 seconds
        if int(self.time_counter * 10) % 20 == 0:
            self.get_logger().info(f'Published skeleton with {len(pose_array.poses)} joints')

def main(args=None):
    rclpy.init(args=args)
    
    skeleton_test_node = SkeletonTestNode()
    
    try:
        rclpy.spin(skeleton_test_node)
    except KeyboardInterrupt:
        print("\nShutting down skeleton test node...")
    finally:
        skeleton_test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
