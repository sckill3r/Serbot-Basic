#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import yaml
import os
import math
import sys


class WaypointFollower(Node):
    """Node for following a sequence of waypoints using Nav2."""

    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Create action client for waypoint following
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')
            
        # Create action client for single pose navigation (as backup)
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
            
        # Publisher for waypoint visualization
        self.marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)
            
        # Wait for action servers
        self.get_logger().info('Waiting for Nav2 waypoint follower...')
        self.follow_waypoints_client.wait_for_server()
        self.get_logger().info('Nav2 waypoint follower connected!')
        
        self.get_logger().info('Waiting for Nav2 navigate to pose...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 navigate to pose connected!')

    def load_waypoints(self, waypoints_file):
        """Load waypoints from a YAML file."""
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f'Waypoints file not found: {waypoints_file}')
            return None
            
        try:
            with open(waypoints_file, 'r') as f:
                waypoints_data = yaml.safe_load(f)
                
            if not isinstance(waypoints_data, dict) or 'waypoints' not in waypoints_data:
                self.get_logger().error('Invalid waypoints file format')
                return None
                
            return waypoints_data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return None

    def create_pose_from_waypoint(self, waypoint, frame_id='map'):
        """Create a PoseStamped message from a waypoint dictionary."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(waypoint['position']['x'])
        pose.pose.position.y = float(waypoint['position']['y'])
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = float(waypoint.get('yaw', 0.0))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def visualize_waypoints(self, poses):
        """Publish markers to visualize the waypoints."""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(poses):
            # Point marker
            marker = Marker()
            marker.header = pose.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
            
            # Text marker with waypoint number
            text_marker = Marker()
            text_marker.header = pose.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose.pose
            text_marker.pose.position.z += 0.3  # Place text above point
            text_marker.text = str(i + 1)
            text_marker.scale.z = 0.3  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = 0
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)

    def follow_waypoints(self, waypoints_file):
        """Follow waypoints from a file using Nav2."""
        # Load waypoints
        waypoints = self.load_waypoints(waypoints_file)
        if not waypoints:
            return False
            
        # Convert waypoints to poses
        poses = [self.create_pose_from_waypoint(wp) for wp in waypoints]
        
        # Visualize waypoints
        self.visualize_waypoints(poses)
        
        # Create and send waypoints goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f'Following {len(poses)} waypoints...')
        
        # Send the goal and wait for result
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints goal was rejected')
            return False
            
        self.get_logger().info('Waypoints goal accepted, waiting for completion...')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Waypoint following completed with {len(result.missed_waypoints)} missed waypoints')
        
        if not result.missed_waypoints:
            self.get_logger().info('All waypoints reached successfully!')
            return True
        else:
            self.get_logger().warn(f'Missed waypoints: {result.missed_waypoints}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    waypoint_follower = WaypointFollower()
    
    if len(sys.argv) < 2:
        waypoint_follower.get_logger().error('Usage: ros2 run serbot_navigation waypoint_follower WAYPOINTS_FILE')
        waypoint_follower.get_logger().info('Example waypoints file format:')
        waypoint_follower.get_logger().info('waypoints:')
        waypoint_follower.get_logger().info('  - position:')
        waypoint_follower.get_logger().info('      x: 1.0')
        waypoint_follower.get_logger().info('      y: 2.0')
        waypoint_follower.get_logger().info('    yaw: 1.57')
        rclpy.shutdown()
        return
    
    try:
        waypoints_file = sys.argv[1]
        waypoint_follower.follow_waypoints(waypoints_file)
    except Exception as e:
        waypoint_follower.get_logger().error(f'Error: {e}')
    finally:
        waypoint_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
