from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r -v 1 empty.sdf',
            description='Arguments for gz_sim.'
        )
    )

    # Initialize Launch Configurations
    description_package = LaunchConfiguration('description_package')
    gz_args = LaunchConfiguration('gz_args')

    # Path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare(description_package),
        'gazebo',
        'worlds',
        'empty.world'
    ])


    # Path to the ArUco tag model
    aruco_model_path = PathJoinSubstitution([
        FindPackageShare(description_package),
        'gazebo',
        'models',
        'arucotag',
        'model.sdf'
    ])

    # Include Gazebo (Ignition Gazebo Sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': [world_path]}.items(),
    )

    # Spawn the ArUco model into Gazebo
    spawn_aruco = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', aruco_model_path,
            '-name', 'arucotag',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'  # posizione nel mondo
        ],
    )

    bridge_camera = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        'camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    ],
    output='screen'
    )

    return LaunchDescription(declared_arguments + [gazebo, spawn_aruco,bridge_camera])