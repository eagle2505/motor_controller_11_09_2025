#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import serial.tools.list_ports
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # List all available ports
        ports = list(serial.tools.list_ports.comports())
        self.get_logger().info("Available ports:")
        for port in ports:
            self.get_logger().info(f"Found port: {port.device}")

        # Try to open the serial port with retries
        self.serial_port = None
        self.connect_serial()

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/simple_velocity_controller/commands',
            self.listener_callback,
            10)
        # '/simple_velocity_controller/commands',
        self.get_logger().info("Serial bridge initialized and ready!")

    def connect_serial(self):
        retry_count = 0
        while retry_count < 3 and not self.serial_port:
            try:
                self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
                self.get_logger().info("Successfully connected to /dev/ttyACM0")
                # Clear any leftover data
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                time.sleep(2)  # Give Arduino time to reset
                return
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open /dev/ttyACM0: {str(e)}")
                try:
                    self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
                    self.get_logger().info("Successfully connected to /dev/ttyACM1")
                    # Clear any leftover data
                    self.serial_port.reset_input_buffer()
                    self.serial_port.reset_output_buffer()
                    time.sleep(2)  # Give Arduino time to reset
                    return
                except serial.SerialException as e:
                    self.get_logger().error(f"Failed to open /dev/ttyACM1: {str(e)}")
            retry_count += 1
            time.sleep(1)
        
        if not self.serial_port:
            self.get_logger().error("Failed to connect to any serial port after retries")

    def listener_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open, attempting to reconnect...")
            self.connect_serial()
            if not self.serial_port or not self.serial_port.is_open:
                return

        try:
            if len(msg.data) >= 4:
                linear_vel = float(msg.data[0])
                angular_vel = float(msg.data[1])
                linear_vel_2 = float(msg.data[2])
                angular_vel_2 = float(msg.data[3])
                output = f"{linear_vel:.2f},{angular_vel:.2f},{linear_vel_2:.2f},{angular_vel_2:.2f}\n"
                
                # Clear buffers before sending
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                # Send the data
                self.get_logger().info(f"Sending to Arduino: {output.strip()}")
                bytes_written = self.serial_port.write(output.encode())
                self.serial_port.flush()  # Ensure all data is sent
                
                # Wait for and read any response
                time.sleep(0.01)  # Give Arduino time to respond
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response:
                        self.get_logger().info(f"Arduino response: {response}")
            else:
                self.get_logger().warn("Received message with insufficient data")
        except (serial.SerialException, ValueError) as e:
            self.get_logger().error(f"Error in serial communication: {str(e)}")
            self.serial_port = None  # Force reconnection on next message

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 