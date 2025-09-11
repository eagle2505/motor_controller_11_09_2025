#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardMover(Node):
    def __init__(self):
        super().__init__('forward_mover')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer to publish velocity commands at 10Hz
        self.timer = self.create_timer(0.1, self.publish_forward_velocity)  # 10Hz
        
        # Robot movement parameters
        self.linear_speed = 1.0  # m/s - forward speed
        self.angular_speed = 0.0  # rad/s - no turning
        
        self.get_logger().info('ForwardMover node started - Robot will move forward continuously')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        
    def publish_forward_velocity(self):
        """Publish forward velocity command"""
        twist_msg = Twist()
        
        # Set linear velocity (forward)
        twist_msg.linear.x = self.linear_speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        
        # Set angular velocity (no turning)
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_speed
        
        # Publish the message
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Log every 50 messages (5 seconds at 10Hz)
        if hasattr(self, '_msg_counter'):
            self._msg_counter += 1
        else:
            self._msg_counter = 0
            
        if self._msg_counter % 50 == 0:
            self.get_logger().info(f'Moving forward at {self.linear_speed} m/s')
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    
    forward_mover = ForwardMover()
    
    try:
        rclpy.spin(forward_mover)
    except KeyboardInterrupt:
        forward_mover.get_logger().info('Received keyboard interrupt, stopping robot...')
        forward_mover.stop_robot()
    finally:
        forward_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 