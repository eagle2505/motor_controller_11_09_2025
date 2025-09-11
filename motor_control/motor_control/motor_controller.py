#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
import serial
import time
import math
from tf2_ros import TransformBroadcaster


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Parameters (kept from your file, plus a couple new)
        self.declare_parameter("wheel_radius", 0.070)
        self.declare_parameter("wheel_separation_lr", 0.360)   # distance between left/right wheels (m)
        self.declare_parameter("wheel_separation_fb", 0.312)
        self.declare_parameter("encoder_resolution", 751.8)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter('max_linear_speed', 0.5)        # not used for mapping anymore, kept for compatibility
        self.declare_parameter('max_angular_speed', 1.0)       # not used for mapping anymore
        self.declare_parameter('max_wheel_speed', 0.8)         # m/s mapped to full PWM
        self.declare_parameter('max_wheel_pwm', 255)           # Arduino expects -255..255

        # Get parameters
        self.wheel_radius_ = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation_lr_ = float(self.get_parameter("wheel_separation_lr").value)
        self.wheel_separation_fb_ = float(self.get_parameter("wheel_separation_fb").value)
        self.encoder_resolution_ = float(self.get_parameter("encoder_resolution").value)
        serial_port = self.get_parameter("serial_port").value
        baudrate = int(self.get_parameter("baudrate").value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.max_wheel_pwm = int(self.get_parameter('max_wheel_pwm').value)

        # Pub/sub and TF
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Serial
        try:
            self.get_logger().info(f'Attempting to connect to serial port {serial_port}...')
            self.serial_port = serial.Serial(serial_port, baudrate, timeout=0.05)
            time.sleep(2)
            self.get_logger().info('Successfully connected to serial port')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            raise

        # Pose state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Timers
        self.timer = self.create_timer(0.02, self.read_serial_data)   # 50 Hz read
        self.static_timer = self.create_timer(0.1, self.publish_static_odometry)

        # Odometry message and TF (keep your frames)
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = 'odom'
        self.odom_msg_.child_frame_id = 'base_link'

        self.odom_msg_.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]
        self.odom_msg_.twist.covariance = list(self.odom_msg_.pose.covariance)

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = 'odom'
        self.transform_stamped_.child_frame_id = 'base_link'

        self._debug_counter = 0
        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # stop if no cmd in 1s
        self.create_timer(0.1, self.safety_timer)

        self.get_logger().info('MotorController node initialized (differential mode)')

    # Map wheel linear speed (m/s) to PWM
    def speed_to_pwm(self, v):
        if self.max_wheel_speed <= 1e-6:
            return 0
        pwm = int(round((v / self.max_wheel_speed) * self.max_wheel_pwm))
        return max(-self.max_wheel_pwm, min(self.max_wheel_pwm, pwm))

    def cmd_vel_callback(self, msg: Twist):
        # Differential drive mapping from Twist to wheel linear speeds
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        v_left = vx - (wz * self.wheel_separation_lr_ / 2.0)
        v_right = vx + (wz * self.wheel_separation_lr_ / 2.0)

        left_pwm = self.speed_to_pwm(v_left)
        right_pwm = self.speed_to_pwm(v_right)

        # Send to Arduino: V,left_pwm,right_pwm
        cmd = f"V,{left_pwm},{right_pwm}\n"
        try:
            self.serial_port.write(cmd.encode('ascii'))
            self.serial_port.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command: {e}')

        self.last_cmd_time = time.time()

    def safety_timer(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            try:
                self.serial_port.write(b"V,0,0\n")
            except Exception:
                pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        q = Quaternion()
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

        if dt > 0.0:
            c = math.cos(self.theta_)
            s = math.sin(self.theta_)
            self.x_ += (vx * c - vy * s) * dt
            self.y_ += (vx * s + vy * c) * dt
            self.theta_ += omega * dt
            self.theta_ = math.atan2(math.sin(self.theta_), math.cos(self.theta_))

        q = self.euler_to_quaternion(0.0, 0.0, self.theta_)

        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation = q
        self.odom_msg_.twist.twist.linear.x = vx
        self.odom_msg_.twist.twist.linear.y = vy
        self.odom_msg_.twist.twist.angular.z = omega
        self.odom_publisher.publish(self.odom_msg_)

        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation = q
        self.tf_broadcaster.sendTransform(self.transform_stamped_)

        self.prev_time_ = current_time

    def publish_static_odometry(self):
        current_time = self.get_clock().now()
        q = self.euler_to_quaternion(0.0, 0.0, self.theta_)
        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation = q
        self.odom_msg_.twist.twist.linear.x = 0.0
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(self.odom_msg_)

        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation = self.odom_msg_.pose.pose.orientation
        self.tf_broadcaster.sendTransform(self.transform_stamped_)

    def read_serial_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            # Expected: ODOM_LR:v_left,v_right   (m/s)
            if line.startswith('ODOM_LR:'):
                payload = line[len('ODOM_LR:'):]
                parts = payload.split(',')
                if len(parts) >= 2:
                    v_left = float(parts[0])
                    v_right = float(parts[1])

                    vx = 0.5 * (v_left + v_right)
                    vy = 0.0
                    omega = (v_right - v_left) / self.wheel_separation_lr_

                    self.publish_odometry(vx, vy, omega)

                    self._debug_counter += 1
                    if self._debug_counter % 50 == 0:
                        self.get_logger().info(f'LR: vl={v_left:.3f} vr={v_right:.3f} -> vx={vx:.3f} wz={omega:.3f}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')

    def destroy_node(self):
        try:
            self.get_logger().info("Stopping motors and closing serial connection...")
            self.serial_port.write(b"V,0,0\n")
            self.serial_port.flush()
            time.sleep(0.1)
            self.serial_port.close()
        except Exception as e:
            self.get_logger().warn(f"Error during cleanup: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

