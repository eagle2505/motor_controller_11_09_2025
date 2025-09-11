#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import serial
import time
import math
import numpy as np
from tf2_ros import TransformBroadcaster

class MecanumOdometryPublisher(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_publisher')
        
        # Declare parameters
        self.declare_parameter("wheel_radius", 0.070)
        self.declare_parameter("wheel_separation_lr", 0.360)
        self.declare_parameter("wheel_separation_fb", 0.312)
        self.declare_parameter("encoder_resolution", 751.8)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        
        # Get parameters
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_lr_ = self.get_parameter("wheel_separation_lr").get_parameter_value().double_value
        self.wheel_separation_fb_ = self.get_parameter("wheel_separation_fb").get_parameter_value().double_value
        self.encoder_resolution_ = self.get_parameter("encoder_resolution").get_parameter_value().double_value
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        
        # Create publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Create transform broadcaster (for odom->base_footprint)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize serial connection
        try:
            self.get_logger().info(f'Attempting to connect to serial port {serial_port}...')
            self.serial_port = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info('Successfully connected to serial port')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            self.get_logger().error('Make sure Arduino is connected and the port is correct')
            raise e
        
        # Initialize odometry variables
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        # Initialize previous time
        self.prev_time_ = self.get_clock().now()
        
        # Create timer for reading serial data
        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz
        
        # Initialize odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = 'odom'
        self.odom_msg_.child_frame_id = 'base_footprint'
        
        # Initialize covariance matrices
        # High uncertainty in y and theta for mecanum wheels
        self.odom_msg_.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,  # y (higher uncertainty)
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2   # yaw (higher uncertainty)
        ]
        
        self.odom_msg_.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]
        
        # Initialize transform
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = 'odom'
        self.transform_stamped_.child_frame_id = 'base_footprint'

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        q = Quaternion()
        
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def publish_odometry(self, vx, vy, omega):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time_).nanoseconds * 1e-9
        
        # Update pose based on velocities
        if dt > 0:
            # Transform velocities from body frame to world frame
            cos_theta = math.cos(self.theta_)
            sin_theta = math.sin(self.theta_)
            
            # Update position
            self.x_ += (vx * cos_theta - vy * sin_theta) * dt
            self.y_ += (vx * sin_theta + vy * cos_theta) * dt
            self.theta_ += omega * dt
            
            # Normalize theta
            self.theta_ = math.atan2(math.sin(self.theta_), math.cos(self.theta_))
        
        # Convert yaw to quaternion
        q = self.euler_to_quaternion(0.0, 0.0, self.theta_)
        
        # Update odometry message
        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation = q
        
        # Set velocities
        self.odom_msg_.twist.twist.linear.x = vx
        self.odom_msg_.twist.twist.linear.y = vy
        self.odom_msg_.twist.twist.linear.z = 0.0
        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = omega
        
        # Publish odometry message
        self.odom_publisher.publish(self.odom_msg_)
        
        # Update and publish transform (odom -> base_footprint)
        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation = q
        self.tf_broadcaster.sendTransform(self.transform_stamped_)
        
        # Update previous time
        self.prev_time_ = current_time

    def read_serial_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                # Read data from serial port
                data = self.serial_port.readline().decode('utf-8').strip()
                if data:
                    # Check if this is an odometry message
                    if data.startswith('ODOM:'):
                        # Remove the 'ODOM:' prefix
                        data = data[5:]
                        # Parse data
                        parts = data.split(',')
                        if len(parts) == 5:  # We expect 5 values: vx,vy,omega,LEFT,RIGHT
                            try:
                                # Parse the direct velocity measurements from Arduino
                                vx = float(parts[0])
                                vy = float(parts[1])
                                omega = float(parts[2])
                                
                                # Optional: Store encoder positions for debugging
                                LEFT_pos = float(parts[3])
                                RIGHT_pos = float(parts[4])
                                
                                # Publish odometry
                                self.publish_odometry(vx, vy, omega)
                                
                                # Log occasionally for debugging
                                if hasattr(self, '_debug_counter'):
                                    self._debug_counter += 1
                                else:
                                    self._debug_counter = 0
                                    
                                if self._debug_counter % 40 == 0:  # Every 2 seconds at 20Hz
                                    self.get_logger().info(f'Odometry: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}')
                                    
                            except ValueError as e:
                                self.get_logger().warn(f'Error parsing odometry data: {e}')
                        else:
                            self.get_logger().warn(f'Invalid odometry data format: expected 5 values, got {len(parts)}')
                    # Ignore other messages
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except UnicodeDecodeError as e:
            self.get_logger().warn(f'Error decoding serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        odometry_publisher = MecanumOdometryPublisher()
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'odometry_publisher' in locals():
            if hasattr(odometry_publisher, 'serial_port'):
                odometry_publisher.serial_port.close()
            odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 