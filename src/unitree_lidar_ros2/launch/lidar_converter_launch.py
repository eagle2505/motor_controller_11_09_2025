import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Run unitree lidar
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'initialize_type': 2},
                {'work_mode': 0},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6201},
                {'local_ip': '192.168.1.2'},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    # Run point cloud to laser scan converter
    converter_node = Node(
        package='unitree_lidar_ros2',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters= [
                {'min_height': -0.5},
                {'max_height': 0.5},
                {'angle_min': -3.14159},
                {'angle_max': 3.14159},
                {'angle_increment': 0.0174533},  # 1 degree
                {'scan_time': 0.1},
                {'range_min': 0.1},
                {'range_max': 50.0},
                {'use_inf': True},
                {'inf_epsilon': 1.0},
                {'target_frame': "base_link"},
                {'input_topic': "/unilidar/cloud"},
                {'output_topic': "/scan"},
                ]
    )

    return LaunchDescription([lidar_node, converter_node]) 