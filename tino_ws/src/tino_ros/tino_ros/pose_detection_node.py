#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.logging
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Pose, PoseArray
from std_msgs.msg import ColorRGBA
from ultralytics import YOLO
import os
import logging
from ament_index_python.packages import get_package_share_directory

class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        
        # Set logging level to reduce verbosity (ERROR, WARN, INFO, DEBUG)
        # Default to INFO, but can be changed with ROS param
        self.declare_parameter('log_level', 'INFO')
        log_level = self.get_parameter('log_level').get_parameter_value().string_value
        if hasattr(rclpy.logging, log_level):
            self.get_logger().set_level(getattr(rclpy.logging, log_level))
        
        # Configure YOLO model's logging level - suppress the detection logs
        # Set Ultralytics logger to ERROR level to suppress the detection logs
        self.declare_parameter('suppress_yolo_logs', True)
        if self.get_parameter('suppress_yolo_logs').value:
            logging.getLogger("ultralytics").setLevel(logging.ERROR)
            # Also silence other related loggers that might be used by YOLO
            logging.getLogger("yolo").setLevel(logging.ERROR)
            logging.getLogger("yolov8").setLevel(logging.ERROR)
            
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
            
        # New publisher for skeleton keypoints
        self.skeleton_publisher = self.create_publisher(
            MarkerArray,
            '/human_skeleton',
            10)
            
        # New publisher for skeleton poses (for programmatic use)
        self.skeleton_poses_publisher = self.create_publisher(
            PoseArray,
            '/human_skeleton_poses',
            10)
            
        # Initialize variables
        self.latest_image = None
        self.latest_depth_image = None
        self.camera_info = None
        
        # For averaging human positions over time to reduce jitter
        self.position_history = []
        self.history_size = 5
        
        # Define skeleton connections (which keypoints are connected)
        # Based on COCO keypoint format used by YOLO
        self.skeleton_connections = [
            (0, 1),  # nose to left_eye
            (0, 2),  # nose to right_eye
            (1, 3),  # left_eye to left_ear
            (2, 4),  # right_eye to right_ear
            (5, 6),  # left_shoulder to right_shoulder
            (5, 7),  # left_shoulder to left_elbow
            (7, 9),  # left_elbow to left_wrist
            (6, 8),  # right_shoulder to right_elbow
            (8, 10), # right_elbow to right_wrist
            (5, 11), # left_shoulder to left_hip
            (6, 12), # right_shoulder to right_hip
            (11, 12), # left_hip to right_hip
            (11, 13), # left_hip to left_knee
            (13, 15), # left_knee to left_ankle
            (12, 14), # right_hip to right_knee
            (14, 16)  # right_knee to right_ankle
        ]
        
        # Keypoint names for reference
        self.keypoint_names = [
            'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
            'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
            'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
        ]
        
        # Keep initialization logs at INFO level
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
            # Run YOLO detection with verbose=False to reduce log output
            results = self.model(self.latest_image, conf=self.confidence_threshold, verbose=False)
            
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
    
    def get_depth_at_point(self, x, y, window_size=5):
        """Get the median depth value at a specific point with a small window"""
        # Ensure coordinates are within image boundaries
        h, w = self.latest_depth_image.shape
        x = max(0, min(x, w-1))
        y = max(0, min(y, h-1))
        
        # Define the window
        half_size = window_size // 2
        x1 = max(0, x - half_size)
        y1 = max(0, y - half_size)
        x2 = min(w, x + half_size + 1)
        y2 = min(h, y + half_size + 1)
        
        # Extract depth values in the window
        depth_window = self.latest_depth_image[y1:y2, x1:x2]
        
        # Filter out zero or invalid depth values
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) > 0:
            # Use median to be robust against outliers
            return np.median(valid_depths)
        else:
            return 0  # No valid depth values found
    
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
                        
                        # Process keypoints if available (only for the first person detected)
                        if hasattr(result, 'keypoints') and len(result.keypoints) > 0:
                            self.process_skeleton(result.keypoints[i], depth_value, human_position)
            except Exception as e:
                self.get_logger().error(f'Error creating marker for detection {i}: {e}')
                
        if marker_array.markers:
            self.markers_publisher.publish(marker_array)
        
        # Publish human position if detected
        if has_person and human_position is not None:
            self.publish_human_position(human_position)
    
    def process_skeleton(self, keypoints, torso_depth, human_position):
        """Process and publish skeleton keypoints"""
        try:
            # Create marker array for skeleton visualization
            skeleton_markers = MarkerArray()
            
            # Create pose array for skeleton joint positions
            skeleton_poses = PoseArray()
            skeleton_poses.header.frame_id = self.camera_frame
            skeleton_poses.header.stamp = self.get_clock().now().to_msg()
            
            # Get keypoints as numpy array
            if hasattr(keypoints, 'xy'):
                kpts = keypoints.xy.cpu().numpy()[0]  # Extract keypoints for the first person
            
                # Process each keypoint
                joint_positions_3d = []
                for idx, (x, y) in enumerate(kpts):
                    # Ignore invalid keypoints (marked with 0,0 or negative values)
                    if x <= 0 or y <= 0:
                        # Add None for invalid keypoints
                        joint_positions_3d.append(None)
                        continue
                    
                    # Get depth at this keypoint
                    kpt_depth = self.get_depth_at_point(int(x), int(y))
                    
                    # If depth is not available, use torso depth
                    if kpt_depth == 0:
                        kpt_depth = torso_depth
                    
                    # Calculate 3D position of this keypoint
                    x_3d, y_3d, z_3d = self.calculate_3d_position(int(x), int(y), kpt_depth)
                    joint_positions_3d.append((x_3d, y_3d, z_3d))
                    
                    # Create a sphere marker for this keypoint
                    joint_marker = Marker()
                    joint_marker.header.frame_id = self.camera_frame
                    joint_marker.header.stamp = self.get_clock().now().to_msg()
                    joint_marker.id = idx
                    joint_marker.type = Marker.SPHERE
                    joint_marker.action = Marker.ADD
                    
                    joint_marker.pose.position.x = x_3d
                    joint_marker.pose.position.y = y_3d
                    joint_marker.pose.position.z = z_3d
                    joint_marker.pose.orientation.w = 1.0
                    
                    joint_marker.scale.x = 0.05  # Joint size
                    joint_marker.scale.y = 0.05
                    joint_marker.scale.z = 0.05
                    
                    # Color by joint type
                    if idx in [0, 1, 2, 3, 4]:  # Head keypoints
                        joint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                    elif idx in [5, 6, 7, 8, 9, 10]:  # Arm keypoints
                        joint_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                    else:  # Leg keypoints
                        joint_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
                    
                    skeleton_markers.markers.append(joint_marker)
                    
                    # Add to pose array
                    pose = Pose()
                    pose.position.x = x_3d
                    pose.position.y = y_3d
                    pose.position.z = z_3d
                    pose.orientation.w = 1.0
                    skeleton_poses.poses.append(pose)
                    
                    # Add text marker with joint name
                    text_marker = Marker()
                    text_marker.header.frame_id = self.camera_frame
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = idx + 100  # Offset to avoid ID collision
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    text_marker.pose.position.x = x_3d
                    text_marker.pose.position.y = y_3d
                    text_marker.pose.position.z = z_3d + 0.1  # Offset text slightly above joint
                    
                    text_marker.scale.z = 0.05  # Text size
                    text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
                    text_marker.text = self.keypoint_names[idx]
                    
                    skeleton_markers.markers.append(text_marker)
                
                # Create lines connecting the joints according to skeleton structure
                marker_id = 1000  # Starting ID for line markers
                for connection in self.skeleton_connections:
                    idx1, idx2 = connection
                    
                    # Check if both keypoints are valid
                    if (idx1 < len(joint_positions_3d) and idx2 < len(joint_positions_3d) and 
                        joint_positions_3d[idx1] is not None and joint_positions_3d[idx2] is not None):
                        
                        line_marker = Marker()
                        line_marker.header.frame_id = self.camera_frame
                        line_marker.header.stamp = self.get_clock().now().to_msg()
                        line_marker.id = marker_id
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        
                        # Add the two points for this line
                        p1 = Point()
                        p1.x = joint_positions_3d[idx1][0]
                        p1.y = joint_positions_3d[idx1][1]
                        p1.z = joint_positions_3d[idx1][2]
                        
                        p2 = Point()
                        p2.x = joint_positions_3d[idx2][0]
                        p2.y = joint_positions_3d[idx2][1]
                        p2.z = joint_positions_3d[idx2][2]
                        
                        line_marker.points = [p1, p2]
                        
                        # Set width of the line
                        line_marker.scale.x = 0.02
                        
                        # Color by body part
                        if idx1 <= 4 and idx2 <= 4:  # Head connections
                            line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
                        elif idx1 <= 10 and idx2 <= 10:  # Arms and shoulders
                            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                        else:  # Legs
                            line_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
                        
                        skeleton_markers.markers.append(line_marker)
                        marker_id += 1
                
                # Publish skeleton markers
                if skeleton_markers.markers:
                    self.skeleton_publisher.publish(skeleton_markers)
                    
                # Publish skeleton poses
                if skeleton_poses.poses:
                    self.skeleton_poses_publisher.publish(skeleton_poses)
                    
                self.get_logger().debug(f"Published skeleton with {len(joint_positions_3d)} valid keypoints")
        
        except Exception as e:
            self.get_logger().error(f'Error processing skeleton: {str(e)}')
    
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
            
            # Log human position at debug level to reduce verbosity
            self.get_logger().debug(f"Human position: x={avg_x:.2f}m, y={avg_y:.2f}m, z={avg_z:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    pose_detection_node = PoseDetectionNode()
    rclpy.spin(pose_detection_node)
    pose_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()