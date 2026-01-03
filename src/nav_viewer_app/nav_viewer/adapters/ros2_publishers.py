"""
ROS2 Publisher Adapters

Concrete implementations of output ports using ROS2 publishers.
These adapt domain entities to ROS2 messages.
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import OccupancyGrid

from ..domain.entities import Pose2D, NavigationGoal
from ..ports.output_ports import IGoalSender, IPoseSetter, ITeleopController


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return x, y, z, w


class ROS2GoalSender(IGoalSender):
    """ROS2 implementation of goal sender."""
    
    def __init__(self, node: Node, topic: str = "/goal_pose"):
        self._node = node
        self._is_active = False
        self._lock = threading.Lock()
        
        # Use simple QoS depth for compatibility with bt_navigator
        self._publisher = node.create_publisher(
            PoseStamped,
            topic,
            1  # Simple depth, uses default QoS
        )
        node.get_logger().info(f"Publishing to {topic}")
    
    def send_goal(self, goal: NavigationGoal) -> bool:
        """Send a navigation goal."""
        msg = PoseStamped()
        msg.header.frame_id = goal.frame_id
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        msg.pose.position.x = goal.pose.x
        msg.pose.position.y = goal.pose.y
        msg.pose.position.z = 0.0
        
        x, y, z, w = quaternion_from_euler(0, 0, goal.pose.theta)
        msg.pose.orientation.x = x
        msg.pose.orientation.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        
        self._publisher.publish(msg)
        
        with self._lock:
            self._is_active = True
        
        self._node.get_logger().info(
            f"Sent goal: x={goal.pose.x:.2f}, y={goal.pose.y:.2f}, "
            f"theta={goal.pose.theta:.2f}"
        )
        return True
    
    def cancel_goal(self) -> bool:
        """Cancel current navigation (publishing empty goal won't cancel, 
        but this is a placeholder for action client usage)."""
        with self._lock:
            self._is_active = False
        self._node.get_logger().info("Goal cancelled (placeholder)")
        return True
    
    def is_navigation_active(self) -> bool:
        with self._lock:
            return self._is_active


class ROS2PoseSetter(IPoseSetter):
    """ROS2 implementation of initial pose setter."""
    
    def __init__(self, node: Node, topic: str = "/initialpose"):
        self._node = node
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._publisher = node.create_publisher(
            PoseWithCovarianceStamped,
            topic,
            qos
        )
        node.get_logger().info(f"Publishing to {topic}")
    
    def set_initial_pose(
        self, 
        pose: Pose2D, 
        covariance_x: float = 0.25,
        covariance_y: float = 0.25,
        covariance_yaw: float = 0.07
    ) -> None:
        """Set the initial pose estimate."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        
        x, y, z, w = quaternion_from_euler(0, 0, pose.theta)
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        
        # 6x6 covariance matrix (x, y, z, roll, pitch, yaw)
        # Only set x, y, and yaw variances
        covariance = [0.0] * 36
        covariance[0] = covariance_x * covariance_x  # x variance
        covariance[7] = covariance_y * covariance_y  # y variance
        covariance[35] = covariance_yaw * covariance_yaw  # yaw variance
        msg.pose.covariance = covariance
        
        self._publisher.publish(msg)
        self._node.get_logger().info(
            f"Set initial pose: x={pose.x:.2f}, y={pose.y:.2f}, "
            f"theta={pose.theta:.2f}"
        )


class ROS2TeleopController(ITeleopController):
    """ROS2 implementation of teleop controller."""
    
    def __init__(self, node: Node, topic: str = "/cmd_vel"):
        self._node = node
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._publisher = node.create_publisher(
            Twist,
            topic,
            qos
        )
        node.get_logger().info(f"Publishing teleop to {topic}")
    
    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        """Send velocity command."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        self._publisher.publish(msg)
    
    def stop(self) -> None:
        """Stop the robot."""
        self.send_velocity(0.0, 0.0)


class ROS2ZoneMaskPublisher:
    """Publishes restricted zones as an OccupancyGrid mask."""
    
    def __init__(self, node: Node, topic: str = "/keepout_mask"):
        self._node = node
        
        # Transient Local Durability (like maps) so late joiners get it
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._publisher = node.create_publisher(
            OccupancyGrid,
            topic,
            qos
        )
        node.get_logger().info(f"Publishing zone mask to {topic}")

    def publish_mask(self, zones: list, map_metadata: Optional['MapData'] = None) -> None:
        """
        Convert zones to OccupancyGrid and publish.
        
        Args:
            zones: List of RestrictedZone objects
            map_metadata: Current map metadata (width, height, resolution, origin)
                          to align mask with map.
        """
        if not map_metadata:
            return

        grid = OccupancyGrid()
        grid.header.stamp = self._node.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        
        # Copy metadata from current map
        grid.info.resolution = map_metadata.resolution
        grid.info.width = map_metadata.width
        grid.info.height = map_metadata.height
        grid.info.origin.position.x = map_metadata.origin.x
        grid.info.origin.position.y = map_metadata.origin.y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0 # No rotation
        
        # Initialize grid with 0 (Free)
        # 100 = Occupied (Keepout), 0 = Free
        data = [0] * (grid.info.width * grid.info.height)
        
        # Rasterize zones
        # This is a simplified rasterization (bounding box + point check) for now.
        # Ideally we use OpenCV or PIP for all pixels, but python loop is slow.
        # Optimization: Only check bounding box of each zone.
        
        origin_x = map_metadata.origin.x
        origin_y = map_metadata.origin.y
        res = map_metadata.resolution
        width = map_metadata.width
        height = map_metadata.height

        for zone in zones:
            if not zone.is_active or len(zone.points) < 3:
                continue
                
            # Filter Logic:
            # Check bounding box of zone in grid coordinates
            xs = [p[0] for p in zone.points]
            ys = [p[1] for p in zone.points]
            min_x = min(xs); max_x = max(xs)
            min_y = min(ys); max_y = max(ys)
            
            # Convert to grid indices
            min_gx = max(0, int((min_x - origin_x) / res))
            max_gx = min(width - 1, int((max_x - origin_x) / res) + 1)
            min_gy = max(0, int((min_y - origin_y) / res))
            max_gy = min(height - 1, int((max_y - origin_y) / res) + 1)
            
            # Simple PIP implementation for rasterization (reusing local check)
            poly = zone.points
            n = len(poly)
            
            for gy in range(min_gy, max_gy):
                y_world = origin_y + (gy + 0.5) * res
                for gx in range(min_gx, max_gx):
                    x_world = origin_x + (gx + 0.5) * res
                    
                    # Point in Polygon Test
                    inside = False
                    p1x, p1y = poly[0]
                    for i in range(n + 1):
                        p2x, p2y = poly[i % n]
                        if y_world > min(p1y, p2y):
                            if y_world <= max(p1y, p2y):
                                if x_world <= max(p1x, p2x):
                                    if p1y != p2y:
                                        xinters = (y_world - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                                    if p1x == p2x or x_world <= xinters:
                                        inside = not inside
                        p1x, p1y = p2x, p2y
                        
                    if inside:
                        data[gy * width + gx] = 100  # Occupied/Keepout
        
        grid.data = data
        self._publisher.publish(grid)
