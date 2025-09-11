#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import copy

class ScanFrameRemapper(Node):
    def __init__(self):
        super().__init__('scan_frame_remapper')
        
        # Subscribe to the original scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish the remapped scan topic
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan_laser',
            10
        )
        
        self.get_logger().info('Scan frame remapper started. Converting /scan (base_link) to /scan_laser (laser)')

    def scan_callback(self, msg):
        # Create a copy of the message
        remapped_msg = copy.deepcopy(msg)
        
        # Change the frame_id from base_link to laser
        remapped_msg.header.frame_id = 'laser'
        
        # Publish the remapped message
        self.scan_pub.publish(remapped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRemapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 