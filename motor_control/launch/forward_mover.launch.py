from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('motor_control')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Motor controller node
    motor_controller_node = Node(
        package='motor_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Forward mover node
    forward_mover_node = Node(
        package='motor_control',
        executable='forward_mover',
        name='forward_mover',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time)
    
    # Add nodes
    ld.add_action(motor_controller_node)
    ld.add_action(forward_mover_node)
    
    return ld 