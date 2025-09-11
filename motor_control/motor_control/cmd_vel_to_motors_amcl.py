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
        self.declare_parameter('wheel_separation_width', 0.360)
        self.declare_parameter('wheel_separation_length', 0.312)
        self.declare_parameter('min_pwm', 60)  # MOD: New param

        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        self.min_pwm = self.get_parameter('min_pwm').value  # MOD

        # Setup serial connection
        self.serial = serial.Serial(port, baud, timeout=1)
        time.sleep(2)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.get_logger().info('CmdVelToMotors node initialized')

    def apply_deadzone_scaling(self, val):
        """ MOD: Ensures PWM is at least min_pwm if not zero """
        if val == 0:
            return 0
        scaled = int(val * 255)
        if abs(scaled) < self.min_pwm:
            scaled = self.min_pwm * (1 if scaled > 0 else -1)
        return max(min(scaled, 255), -255)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        r = (self.wheel_separation_width + self.wheel_separation_length) / 2.0

        fl = (linear_x - linear_y - angular_z * r)
        fr = (linear_x + linear_y + angular_z * r)
        bl = (linear_x + linear_y - angular_z * r)
        br = (linear_x - linear_y + angular_z * r)

        # Normalize wheel speeds
        max_wheel_speed = max(abs(fl), abs(fr), abs(bl), abs(br))
        if max_wheel_speed > 1.0:
            fl /= max_wheel_speed
            fr /= max_wheel_speed
            bl /= max_wheel_speed
            br /= max_wheel_speed

        # Apply scaling with minimum threshold
        fl_cmd = self.apply_deadzone_scaling(fl)
        fr_cmd = self.apply_deadzone_scaling(fr)
        bl_cmd = self.apply_deadzone_scaling(bl)
        br_cmd = self.apply_deadzone_scaling(br)

        command = f"V,{fl_cmd},{fr_cmd},{bl_cmd},{br_cmd}\n"
        self.serial.write(command.encode('utf-8'))

    def destroy_node(self):
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
