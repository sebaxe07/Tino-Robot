import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters=[{'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True,
                 'localization':True,
                 'database_path': os.path.expanduser('~/rtabmap.db'),
                 'Mem/IncrementalMemory': "False",   # disable mapping
                 'Rtabmap/DetectionRate': "3.0"  
}]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)
    ])
