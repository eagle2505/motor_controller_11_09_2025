from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # LiDAR parameters
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    # Get package share directory
    pkg_share = FindPackageShare('motor_control').find('motor_control')
    
    # Set paths
    urdf_model_path = PathJoinSubstitution([pkg_share, 'description', 'robot.urdf.xacro'])
    default_slam_params_path = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_path,
        description='Full path to the ROS2 parameters file for SLAM')

    # Declare LiDAR arguments
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')
        
    declare_serial_baudrate_cmd = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')

    # Start robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model_path])
        }]
    )

    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Static transforms for proper frame connections
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    # Transform from base_link to laser frame (adjust height and orientation as needed)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.305', '--yaw', '3.1416', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )

    # Transform from base_link to unilidar_lidar frame to connect your existing LiDAR
    static_tf_unilidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_unilidar',
        arguments=['--x', '0', '--y', '0', '--z', '0.305', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'unilidar_lidar'],
        output='screen'
    )



    # Use the combined motor controller node (same as localization.launch.py)
    motor_controller_node = Node(
        package='motor_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    # Start teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )

    # Add slam_toolbox node using online_async_launch.py
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    # Add RViz node with delay to ensure TF transforms are ready
    rviz_config_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'slam.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_serial_baudrate_cmd)

    # Add nodes to launch description in the correct order
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(static_tf_laser)
    ld.add_action(static_tf_unilidar)
    ld.add_action(motor_controller_node)
    ld.add_action(teleop_node)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(rviz_node)

    return ld 