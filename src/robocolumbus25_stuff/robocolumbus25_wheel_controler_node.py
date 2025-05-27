"""
This robocolumbus wheel controller ROS2 is for controlling the 
1/6 scale model jeep wheel velocity and front steering
The steering model is Ackerman type
The "/cmd_vel" topic is subcribed to and the linear.x and angular.z 
velocity commands are used to create the jeep rear wheel velocity 
and the front wheel steering angle via a serial interface
The serial interface is TBD
The wheel diameter is TBD
The wheel base is TBD
The wheel spacing is TBD
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import serial
import json

class WheelControllerNode(Node):
    def __init__(self):
        super().__init__('robocolumbus25_wheel_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # Example parameters (replace with actual values)
        self.wheel_base = 0.5  # meters, TBD
        self.wheel_diameter = 0.15  # meters, TBD

        # Serial port configuration (update as needed)
        self.serial_port = 'COM3'  # Change to your serial port
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute rear wheel velocity (m/s)
        wheel_velocity = linear_x

        # Compute front wheel steering angle (Ackermann)
        if angular_z != 0 and linear_x != 0:
            turning_radius = linear_x / angular_z
            steering_angle = math.atan(self.wheel_base / turning_radius)
        else:
            steering_angle = 0.0

        self.get_logger().info(
            f"Rear wheel velocity: {wheel_velocity:.3f} m/s, "
            f"Front steering angle: {math.degrees(steering_angle):.3f} deg"
        )

        # TODO: Send commands over serial interface
        # Send commands over serial interface as JSON
        if self.ser and self.ser.is_open:
            command = {
                "wheel_velocity": wheel_velocity,
                "steering_angle": steering_angle  # radians
            }
            try:
                json_cmd = json.dumps(command) + '\n'
                self.ser.write(json_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()