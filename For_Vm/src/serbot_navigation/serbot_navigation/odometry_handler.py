#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
import numpy as np
from tf2_ros import StaticTransformBroadcaster
import sys
import traceback

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odometry_handler')
        
        # Declare parameters
        self.declare_parameter('odom_input_topic', '/odom')
        self.declare_parameter('odom_output_topic', '/odom_corrected')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        self.odom_input_topic = self.get_parameter('odom_input_topic').value
        self.odom_output_topic = self.get_parameter('odom_output_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Create a subscriber to the odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_input_topic,
            self.odom_callback,
            10)
        
        # Create a subscriber to the initialpose topic
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10)
        
        # Create a publisher for the modified odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_output_topic,
            10)
        
        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a static transform broadcaster for map->odom
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Initialize the transform from map to odom
        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = 'map'
        self.map_to_odom.child_frame_id = 'odom'
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.x = 0.0
        self.map_to_odom.transform.rotation.y = 0.0
        self.map_to_odom.transform.rotation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0
        
        self.get_logger().info(f'OdomRepublisher started. Subscribing to {self.odom_input_topic} and publishing to {self.odom_output_topic}')
        
        # Publish the initial map->odom transform
        self.publish_map_to_odom()
        
        # Store the initial pose offset
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        
        self.get_logger().info('Odometry republisher started')
    
    def initialpose_callback(self, msg):
        self.get_logger().info('Received initial pose')
        
        # Extract the pose from the message
        pose = msg.pose.pose
        
        # Update the map to odom transform
        self.map_to_odom.transform.translation.x = pose.position.x
        self.map_to_odom.transform.translation.y = pose.position.y
        self.map_to_odom.transform.translation.z = pose.position.z
        self.map_to_odom.transform.rotation = pose.orientation
        
        # Publish the updated transform
        self.publish_map_to_odom()
        
        self.get_logger().info(f'Updated map->odom transform: ({self.map_to_odom.transform.translation.x}, {self.map_to_odom.transform.translation.y})')
    
    def odom_callback(self, msg):
        # Create a transform for odom->base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # Copy the pose from the odometry message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Republish the odometry message with the map frame
        corrected_odom = Odometry()
        corrected_odom.header.stamp = self.get_clock().now().to_msg()
        corrected_odom.header.frame_id = 'map'
        corrected_odom.child_frame_id = 'base_link'
        
        # Copy the pose and twist
        corrected_odom.pose = msg.pose
        corrected_odom.twist = msg.twist
        
        # Publish the corrected odometry
        self.odom_pub.publish(corrected_odom)
    
    def publish_map_to_odom(self):
        # Update the timestamp
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the static transform
        self.static_broadcaster.sendTransform(self.map_to_odom)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomRepublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in odometry_handler: {str(e)}', file=sys.stderr)
        traceback.print_exc()
    finally:
        if rclpy.ok():
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
