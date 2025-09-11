#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class PSControllerTeleop(Node):

    def __init__(self):
        super().__init__('ps_controller_teleop')
        # Publisher for Arduino commands
        self.arduino_publisher = self.create_publisher(
            Float64MultiArray,
            '/simple_velocity_controller/commands',
            10
        )
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        # Scale factors for fine-tuning controller sensitivity
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.linear_scale_2 = 1.0
        self.angular_scale_2 = 1.0
        self.get_logger().info("PS Controller Teleop Node has started.")

    def joy_callback(self, msg: Joy):
        # First stick (left stick):
        # axes[1]: Y-axis (up/down) for linear velocity (forward/backward)
        # axes[0]: X-axis (left/right) for angular velocity (turning)
        linear_vel = int(self.linear_scale * msg.axes[1] * 10) / 10.0
        angular_vel = int(self.angular_scale * msg.axes[0] * 10) / 10.0
        
        # Second stick (right stick):
        # axes[3]: Y-axis (up/down) for linear velocity 2 (side movement)
        # axes[2]: X-axis (left/right) for angular velocity 2 (diagonal movement)
        linear_vel_2 = int(self.linear_scale_2 * msg.axes[3] * 10) / 10.0
        angular_vel_2 = int(self.angular_scale_2 * msg.axes[2] * 10) / 10.0
        
        # Create Float64MultiArray message for Arduino
        arduino_msg = Float64MultiArray()
        arduino_msg.data = [linear_vel, angular_vel, linear_vel_2, angular_vel_2]
        
        # Publish message
        self.arduino_publisher.publish(arduino_msg)
        
        # Log the values being sent
        self.get_logger().info(f'Published: [{linear_vel:.1f}, {angular_vel:.1f}, {linear_vel_2:.1f}, {angular_vel_2:.1f}]')

def main(args=None):
    rclpy.init(args=args)
    node = PSControllerTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 