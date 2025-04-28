#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Using String for simple speed data
import serial
import threading  # To handle reading from serial in a separate thread

# Open Serial connection to Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Ensure timeout is set


class TeleopToSerial(Node):
    def __init__(self):
        super().__init__('teleop_to_serial')
        
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # Publisher for speed feedback
        self.speed_publisher = self.create_publisher(String, '/speed', 10)
        
        # Start a background thread to read speed feedback from Arduino
        self.serial_thread = threading.Thread(target=self.read_serial_feedback, daemon=True)
        self.serial_thread.start()

    def listener_callback(self, msg):
        """Send linear and angular velocity commands to Arduino"""
        linear_x = round(msg.linear.x, 2)
        angular_z = round(msg.angular.z, 2)
        command = f"{linear_x} {angular_z}\n"
        ser.write(command.encode())  # Send command to Arduino
        self.get_logger().info(f"Sent: {command.strip()}")

    def read_serial_feedback(self):
        """Continuously read speed feedback from Arduino"""
        while rclpy.ok():
            try:
                line = ser.readline().decode().strip()  # Read line from Arduino
                if line.startswith("speed "):  # Check if it's a speed message
                    data = line.split()  # Split the message
                    if len(data) == 4:  # Ensure correct format
                        act_speedL = data[1]
                        act_speedR = data[2]
                        loop_time = data[3]

                        # Format data into a ROS2 message
                        speed_msg = String()
                        speed_msg.data = f"L: {act_speedL}, R: {act_speedR}, LoopTime: {loop_time}"
                        self.speed_publisher.publish(speed_msg)  # Publish speed data
                        self.get_logger().info(f"Published: {speed_msg.data}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ser.close()  # Close serial connection

if __name__ == '__main__':
    main()

