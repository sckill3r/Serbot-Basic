#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import sys

class BaseFootprintPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'base_footprint')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for publishing transforms
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.broadcast_transform)
        
        self.get_logger().info(f'BaseFootprintPublisher started. Publishing {self.parent_frame} -> {self.child_frame} at {self.publish_rate} Hz')

    def broadcast_transform(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            
            # No translation
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # No rotation (identity quaternion)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {str(e)}')

def main(args=None):
    try:
        rclpy.init(args=args)
        node = BaseFootprintPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in tf_publisher: {str(e)}', file=sys.stderr)
    finally:
        if rclpy.ok():
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
