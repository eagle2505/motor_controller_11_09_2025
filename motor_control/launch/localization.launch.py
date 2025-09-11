from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # LiDAR parameters
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    # Get package share directory
    pkg_share = FindPackageShare('motor_control').find('motor_control')

    # Set paths
    urdf_model_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    default_map_path = os.path.join(pkg_share, 'maps', 'map1.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'localization.rviz')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=default_map_path)
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_path)
    declare_serial_port_cmd = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM3')
    declare_serial_baudrate_cmd = DeclareLaunchArgument('serial_baudrate', default_value='115200')
    declare_frame_id_cmd = DeclareLaunchArgument('frame_id', default_value='laser')
    declare_inverted_cmd = DeclareLaunchArgument('inverted', default_value='false')
    declare_angle_compensate_cmd = DeclareLaunchArgument('angle_compensate', default_value='true')
    declare_rviz_config_cmd = DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path)
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='true')

    # Nodes
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )



    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['--x', '0', '--y', '0', '--z', '0.305', '--yaw', '3.1416', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )

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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Use simpler behavior trees without recovery actions to avoid plugin loading issues
    bt_nav_to_pose_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_distance.xml'
    ])
    
    # For navigate_through_poses, we'll use a simple one without recovery actions
    bt_nav_through_poses_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_distance.xml'
    ])

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'default_nav_to_pose_bt_xml': bt_nav_to_pose_xml,
             'default_nav_through_poses_bt_xml': bt_nav_through_poses_xml,
             'use_sim_time': use_sim_time}
        ]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Behavior server for recovery behaviors (spin, backup, etc.)
    # Temporarily commented out due to plugin loading issues
    # behaviors_node = Node(
    #     package='nav2_behaviors',
    #     executable='behavior_server',
    #     name='behavior_server',
    #     output='screen',
    #     parameters=[params_file, {'use_sim_time': use_sim_time}]
    # )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # Combined motor controller node (replaces odometry_publisher and cmd_vel_to_motors)
    motor_controller_node = Node(
        package='motor_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )
    
    # RPLidar A1M8 node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'channel_type': 'serial',
            'topic_name': 'scan',
        }],
    )

    rviz_config_path = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_serial_baudrate_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_inverted_cmd)
    ld.add_action(declare_angle_compensate_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(motor_controller_node)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(static_tf_laser)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(rplidar_node)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    # ld.add_action(behaviors_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)

    return ld
