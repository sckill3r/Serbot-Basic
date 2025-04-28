#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import sys
import math


class NavigationInterface(Node):
    """Simple interface for sending navigation goals to Nav2."""

    def __init__(self):
        super().__init__('nav_interface')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Navigation interface initialized. Waiting for Nav2 action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')

    def send_goal(self, x, y, theta=0.0, frame_id='map'):
        """Send a navigation goal to the specified pose."""
        self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, theta={theta}')
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (simple yaw rotation)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)
        
        # Create action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal
        self.get_logger().info('Waiting for navigation result...')
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            return False
            
        self.get_logger().info('Goal accepted by the action server, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
            return True
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    nav_interface = NavigationInterface()
    
    if len(sys.argv) < 3:
        nav_interface.get_logger().error('Usage: ros2 run serbot_navigation nav_interface X Y [THETA]')
        nav_interface.get_logger().error('  X, Y: Goal coordinates in meters')
        nav_interface.get_logger().error('  THETA: Optional goal orientation in radians')
        rclpy.shutdown()
        return
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        nav_interface.send_goal(x, y, theta)
    except ValueError as e:
        nav_interface.get_logger().error(f'Invalid arguments: {e}')
    except Exception as e:
        nav_interface.get_logger().error(f'Error: {e}')
    finally:
        nav_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
