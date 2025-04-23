#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import ColorRGBA
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Define parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.declare_parameter('image_topic', '/right/image_rect')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('depth_info_topic', '/stereo/camera_info')
        
        # Add camera frame parameter with default to oak's right camera frame
        self.declare_parameter('camera_frame', 'oak_right_camera_optical_frame')
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_info_topic = self.get_parameter('depth_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Load YOLO model
        try:
            package_share_dir = get_package_share_directory('tino_ros')
            model_path = os.path.join(package_share_dir, 'resource', 'yolo11n-pose.engine')
            self.get_logger().info(f'Loading model from: {model_path}')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise
        
        # Subscribe to camera topics
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10)
            
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.camera_info_callback,
            10)
            
        # Publishers
        self.detection_image_publisher = self.create_publisher(
            Image,
            '/pose_detection/image_raw',
            10)
            
        self.markers_publisher = self.create_publisher(
            MarkerArray,
            '/pose_detection/markers',
            10)
        
        # Publish detected human positions
        self.human_position_publisher = self.create_publisher(
            PoseStamped,
            '/human_position',
            10)
            
        # Initialize variables
        self.latest_image = None
        self.latest_depth_image = None
        self.camera_info = None
        
        # For averaging human positions over time to reduce jitter
        self.position_history = []
        self.history_size = 5
        
        self.get_logger().info(f'Pose detection node initialized with image topic: {self.image_topic}')
        self.get_logger().info(f'Using camera frame: {self.camera_frame}')
        
    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            # Convert mono8 to bgr8 for YOLO processing if needed
            self.latest_image = cv2.cvtColor(self.latest_image, cv2.COLOR_GRAY2BGR)
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
            
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {e}')
            
    def process_images(self):
        if self.latest_image is None or self.latest_depth_image is None or self.camera_info is None:
            return
            
        try:
            # Run YOLO detection
            results = self.model(self.latest_image, conf=self.confidence_threshold)
            
            # Visualize results
            annotated_image = results[0].plot()
            
            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.camera_frame
            self.detection_image_publisher.publish(img_msg)
            
            # Create markers for visualization and publish human positions
            self.process_detections(results[0])
                
        except Exception as e:
            self.get_logger().error(f'Error in pose detection: {e}')
    
    def get_median_depth(self, x1, y1, x2, y2, padding_factor=0.25):
        """Get the median depth value in the central region of the detected person's bounding box"""
        # Calculate the center region with padding
        width = x2 - x1
        height = y2 - y1
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        pad_x = int(width * padding_factor)
        pad_y = int(height * padding_factor)
        
        # Define the central region
        roi_x1 = max(0, center_x - pad_x)
        roi_y1 = max(0, center_y - pad_y)
        roi_x2 = min(self.latest_depth_image.shape[1], center_x + pad_x)
        roi_y2 = min(self.latest_depth_image.shape[0], center_y + pad_y)
        
        # Extract depth values in the region
        depth_roi = self.latest_depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
        
        # Filter out zero or invalid depth values
        valid_depths = depth_roi[depth_roi > 0]
        
        if len(valid_depths) > 0:
            # Use median to be robust against outliers
            return np.median(valid_depths)
        else:
            return 0  # No valid depth values found
    
    def calculate_3d_position(self, x, y, depth):
        """Calculate the 3D position based on pixel coordinates and depth"""
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        # Convert from image coordinates to 3D coordinates
        z = depth / 1000.0  # Convert to meters
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        
        return x_3d, y_3d, z
    
    def process_detections(self, result):
        marker_array = MarkerArray()
        has_person = False
        human_position = None
        
        # Process detections
        for i, detection in enumerate(result.boxes):
            try:
                # Get bounding box
                x1, y1, x2, y2 = map(int, detection.xyxy[0])
                class_id = int(detection.cls[0])
                confidence = float(detection.conf[0])
                
                # Get class name
                class_name = result.names[class_id]
                
                if class_name == "person":
                    has_person = True
                    
                    # Get center point of detection
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Get median depth value in the central region
                    depth_value = self.get_median_depth(x1, y1, x2, y2)
                    
                    if depth_value > 0:
                        # Calculate 3D position
                        x_3d, y_3d, z_3d = self.calculate_3d_position(center_x, center_y, depth_value)
                        
                        # Store human position
                        human_position = (x_3d, y_3d, z_3d)
                        
                        # Create text marker to display the position
                        marker = Marker()
                        marker.header.frame_id = self.camera_frame
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.id = i
                        marker.type = Marker.TEXT_VIEW_FACING
                        marker.action = Marker.ADD
                        
                        marker.pose.position.x = x_3d
                        marker.pose.position.y = y_3d
                        marker.pose.position.z = z_3d
                        
                        marker.scale.z = 0.1  # Text size
                        
                        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                        marker.text = f"{class_name}: {confidence:.2f}, Dist: {z_3d:.2f}m"
                        
                        marker_array.markers.append(marker)
                        
                        # Add a second marker to visualize the person's position
                        position_marker = Marker()
                        position_marker.header.frame_id = self.camera_frame
                        position_marker.header.stamp = self.get_clock().now().to_msg()
                        position_marker.id = i + 1000  # Ensure unique ID
                        position_marker.type = Marker.SPHERE
                        position_marker.action = Marker.ADD
                        
                        position_marker.pose.position.x = x_3d
                        position_marker.pose.position.y = y_3d
                        position_marker.pose.position.z = z_3d
                        
                        position_marker.scale.x = 0.2
                        position_marker.scale.y = 0.2
                        position_marker.scale.z = 0.2
                        
                        position_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                        
                        marker_array.markers.append(position_marker)
            except Exception as e:
                self.get_logger().error(f'Error creating marker for detection {i}: {e}')
                
        if marker_array.markers:
            self.markers_publisher.publish(marker_array)
        
        # Publish human position if detected
        if has_person and human_position is not None:
            self.publish_human_position(human_position)
    
    def publish_human_position(self, position):
        """Publish the human position with smoothing"""
        # Add current position to history
        self.position_history.append(position)
        
        # Keep history at maximum size
        if len(self.position_history) > self.history_size:
            self.position_history.pop(0)
        
        # Calculate average position
        if self.position_history:
            avg_x = sum(pos[0] for pos in self.position_history) / len(self.position_history)
            avg_y = sum(pos[1] for pos in self.position_history) / len(self.position_history)
            avg_z = sum(pos[2] for pos in self.position_history) / len(self.position_history)
            
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.camera_frame
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            pose_msg.pose.position.x = avg_x
            pose_msg.pose.position.y = avg_y
            pose_msg.pose.position.z = avg_z
            
            # Set orientation to identity quaternion (no rotation)
            pose_msg.pose.orientation.w = 1.0
            
            # Publish position
            self.human_position_publisher.publish(pose_msg)
            
            # Log human position at reasonable intervals
            self.get_logger().info(f"Human position: x={avg_x:.2f}m, y={avg_y:.2f}m, z={avg_z:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    pose_detection_node = PoseDetectionNode()
    rclpy.spin(pose_detection_node)
    pose_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()