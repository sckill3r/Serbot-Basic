#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math


class SerbotController(Node):
    def __init__(self):
        super().__init__('serbot_controller')
        
        # Subscribe to /speed topic
        self.subscription = self.create_subscription(String, '/speed', self.speed_callback, 10)
        
        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Robot parameters
        self.wheel_base = 0.38  # Distance between wheels in meters
        self.wheel_radius = 0.082  # Wheel radius in meters

    def speed_callback(self, msg):
        try:
            # Extract left and right wheel speeds from string message
            data = msg.data.split(', ')
            L_speed = float(data[0].split(': ')[1])
            R_speed = float(data[1].split(': ')[1])
            loop_time = float(data[2].split(': ')[1])
        except Exception as e:
            self.get_logger().error(f"Error parsing speed message: {e}")
            return
        
        # Compute linear and angular velocities
        v = (L_speed + R_speed) / 2.0  # Linear velocity
        w = (R_speed - L_speed) / self.wheel_base  # Angular velocity
        
        # Compute change in position and orientation
        dt = loop_time  # Time step from feedback
        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt
        dtheta = w * dt
        
        # Update pose
        self.x += dx
        self.y += dy
        self.theta += dtheta
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0 
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_publisher.publish(odom)
        
        # Publish TF transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y  # Fix: Add y component
        transform.transform.translation.z = 0.0
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = SerbotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
