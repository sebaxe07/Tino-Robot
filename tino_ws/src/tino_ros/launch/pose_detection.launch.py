from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for pose detection'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/right/image_rect',
        description='The camera image topic to use for pose detection'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/stereo/depth',
        description='The depth image topic to use for 3D position estimation'
    )
    
    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/stereo/camera_info',
        description='Camera info for the depth image'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='oak_right_camera_optical_frame',
        description='The camera frame ID to use for publishing visualization markers'
    )
    
    pose_detection_node = Node(
        package='tino_ros',
        executable='pose_detection',
        name='pose_detection',
        output='screen',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'image_topic': LaunchConfiguration('image_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            'camera_frame': LaunchConfiguration('camera_frame')
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('depth', LaunchConfiguration('depth_topic')),
            ('depth_camera_info', LaunchConfiguration('depth_info_topic'))
        ]
    )
    
    return LaunchDescription([
        confidence_threshold_arg,
        image_topic_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        camera_frame_arg,
        pose_detection_node
    ])