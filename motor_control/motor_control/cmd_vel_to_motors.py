#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import math

class CmdVelToMotors(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motors')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('wheel_separation_width', 0.360)  # meters (left-right distance)
        self.declare_parameter('wheel_separation_length', 0.312)  # meters (front-back distance)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        
        # Setup serial connection
        self.serial = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('CmdVelToMotors node initialized')
        
    def cmd_vel_callback(self, msg):
        # Extract velocities
        linear_x = msg.linear.x   # Forward/backward
        linear_y = msg.linear.y   # Left/right
        angular_z = msg.angular.z  # Rotation
        
        # Calculate wheel velocities for mecanum drive
        # Normalize to motor speed range (0-255)
        max_linear = self.max_linear_speed
        max_angular = self.max_angular_speed
        
        # Mecanum wheel velocity calculations
        fl = (linear_x - linear_y - angular_z * (self.wheel_separation_width + self.wheel_separation_length) / 2.0)
        fr = (linear_x + linear_y + angular_z * (self.wheel_separation_width + self.wheel_separation_length) / 2.0)
        bl = (linear_x + linear_y - angular_z * (self.wheel_separation_width + self.wheel_separation_length) / 2.0)
        br = (linear_x - linear_y + angular_z * (self.wheel_separation_width + self.wheel_separation_length) / 2.0)
        
        # Scale to motor commands (0-255)
        max_wheel_speed = max(abs(fl), abs(fr), abs(bl), abs(br))
        if max_wheel_speed > 1.0:
            fl /= max_wheel_speed
            fr /= max_wheel_speed
            bl /= max_wheel_speed
            br /= max_wheel_speed
            
        # Convert to motor commands
        fl_cmd = int(fl * 255)
        fr_cmd = int(fr * 255)
        bl_cmd = int(bl * 255)
        br_cmd = int(br * 255)
        
        # Send to Arduino
        command = f"V,{fl_cmd},{fr_cmd},{bl_cmd},{br_cmd}\n"
        self.serial.write(command.encode('utf-8'))
        
    def destroy_node(self):
        # Stop motors before shutting down
        self.serial.write(b"V,0,0,0,0\n")
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()