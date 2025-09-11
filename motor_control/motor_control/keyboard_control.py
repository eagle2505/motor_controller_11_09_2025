#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import sys
import tty
import termios
import select
import time
import threading
import curses

# import termios

# # Disable terminal flow control (XON/XOFF)
# attrs = termios.tcgetattr(sys.stdin)
# attrs[3] = attrs[3] & ~termios.IXON
# termios.tcsetattr(sys.stdin, termios.TCSANOW, attrs)



class motorcontrol(Node):

    def __init__(self):
        super().__init__("four_motor_controller")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.running = True
        self.key_thread = threading.Thread(target=self.key_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

    def key_loop(self):
        print("Controls: [↑] forward | [↓] backward | [→] right crab walk | [←] left crab walk\n")
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            last_cmd = ''
            last_time = time.time()
            timeout_duration = 0.5

            while self.running:
                if self.kbhit():
                    c = sys.stdin.read(1)
                    
                    if c == '\x1b':  # Start of escape sequence
                        c2 = sys.stdin.read(1)
                        if c2 == '[':
                            c3 = sys.stdin.read(1)
                            if c3 == 'A':  # Up arrow
                                self.send_command("U\n")
                                last_cmd = 'U'
                            elif c3 == 'B':  # Down arrow
                                self.send_command("B\n")
                                last_cmd = 'B'
                            elif c3 == 'C':  # Right arrow
                                self.send_command("R\n")
                                last_cmd = 'R'
                            elif c3 == 'D':  # Left arrow
                                self.send_command("L\n")
                                last_cmd = 'L'
                    elif c == ' ':  # Space bar
                        self.send_command("Z\n")
                        last_cmd = 'Z'
                    elif c == 'd':  #to the left
                        self.send_command("D\n")
                        last_cmd = 'D'
                    elif c == 'f':  # to the right
                        self.send_command("F\n")
                        last_cmd = 'F'
                    elif c == 'q':  # D: left-up
                        self.send_command("Q\n")
                        last_cmd = 'Q'
                    elif c == 'w':  # D: right - up
                        self.send_command("W\n")
                        last_cmd = 'W'
                    elif c == 'a':  # D: left-Down
                        self.send_command("A\n")
                        last_cmd = 'A'
                    elif c == 's':  # D: Right Down
                        self.send_command("S\n")
                        last_cmd = 'S'
                    elif c == 'q':  # Quit
                        self.send_command("Z\n")
                        self.running = False
                        break

                    last_time = time.time()

                if last_cmd in ['U', 'B', 'R', 'L', 'D', 'F', 'Q', 'W', 'A', 'S'] and (time.time() - last_time > timeout_duration):
                    self.send_command("Z\n")
                    last_cmd = 'Z'

                time.sleep(0.05)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



    def kbhit(self):
        # Non-blocking keyboard check
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []

    def send_command(self, cmd):
        self.arduino_.write(cmd.encode("utf-8"))
        self.arduino_.flush()
        self.get_logger().info(f"Sent: {cmd.strip()}")

    def destroy_node(self):
        self.running = False
        if self.key_thread.is_alive():
            self.key_thread.join()
        self.arduino_.write(b"Z\n")
        self.arduino_.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = motorcontrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
