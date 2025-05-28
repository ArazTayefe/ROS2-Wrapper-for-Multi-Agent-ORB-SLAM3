# all_drones_orbslam3.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='mono',
            name='alpha',
            parameters=[
                {'vocab_path': '/home/walle/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt'},
                {'config_path': '/home/walle/ros2_ws/src/orbslam3_ros2/config/monocular/TUM1.yaml'},
                {'use_viewer': False}
            ],
            remappings=[
                ('camera', '/Alpha/image_raw')
            ]
        ),
        Node(
            package='orbslam3',
            executable='mono',
            name='bravo',
            parameters=[
                {'vocab_path': '/home/walle/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt'},
                {'config_path': '/home/walle/ros2_ws/src/orbslam3_ros2/config/monocular/TUM1.yaml'},
                {'use_viewer': False}
            ],
            remappings=[
                ('camera', '/Bravo/image_raw')
            ]
        ),
        Node(
            package='orbslam3',
            executable='mono',
            name='charlie',
            parameters=[
                {'vocab_path': '/home/walle/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt'},
                {'config_path': '/home/walle/ros2_ws/src/orbslam3_ros2/config/monocular/TUM1.yaml'},
                {'use_viewer': False}
            ],
            remappings=[
                ('camera', '/Charlie/image_raw')
            ]
        )
    ])
