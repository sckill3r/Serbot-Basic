#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# Open Serial connection to Arduino
ser = serial.Serial('/dev/ttyACM0', 115200)  # Change port if necessary

class TeleopToSerial(Node):
    def __init__(self):
        super().__init__('teleop_to_serial')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        linear_x = round(msg.linear.x, 2)
        angular_z = round(msg.angular.z, 2)
        command = f"{linear_x} {angular_z}\n"
        ser.write(command.encode())  # Send to Arduino
        self.get_logger().info(f"Sent: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    ser.close()

if __name__ == '__main__':
    main()
