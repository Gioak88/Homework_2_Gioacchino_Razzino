from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'camera_frame': 'camera_link',   # coincide con il frame della camera nel SDF
                'marker_size': 0.1,              # dimensione del marker in metri
                'marker_dictionary': 'ARUCO_MIP_36h12'
            }]
        )
    ])
