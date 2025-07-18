# """
# ===================================================================
# |                                                                 |
# |  COPYRIGHT NOTICE                                               |
# |                                                                 |
# |  This code is developed by OpenIoT - Open Platforms and         |
# |  Enabling Technologies for the Internet of Things.              |
# |                                                                 |
# |  Developer: Farhad Shamsfakhr, PhD                              |
# |  Affiliation: Fondazione Bruno Kessler (FBK)                    |
# |  Address: via Sommarive, 18 38123 Povo, Trento (Italy)          |
# |                                                                 |
# |  This project is sponsored by the Agri-Food project,            |
# |  coordinated by FONDAZIONE BRUNO KESSLER.                       |
# |  Funded under the Digital Europe Programme (DIGITAL).           |                        |
# |                                                                 |
# |  © [2024] Farhad Shamsfakhr, PhD. All rights reserved.          |
# |                                                                 |
# ===================================================================
# """
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwb_positioning',
            executable='main.py',
            name='uwb_positioning',
            output='screen',
            parameters=[
                {'serial_port_name': '/dev/ttyACM0'},
                {'serial_baud_rate': 115200},
            ],
        ),
    ])
