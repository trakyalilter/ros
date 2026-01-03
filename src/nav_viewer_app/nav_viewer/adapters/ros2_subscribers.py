"""
ROS2 Subscriber Adapters

Concrete implementations of input ports using ROS2 subscriptions.
These adapt ROS2 messages to domain entities.
"""

import threading
import time
import math
from typing import Callable, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
    qos_profile_sensor_data
)

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

from ..domain.entities import (
    MapData, LaserScanData, Pose2D, PathData, 
    CostmapData, RobotState, NavigationStatus
)
from ..ports.input_ports import (
    IMapReceiver, ILaserReceiver, IPoseReceiver, 
    IPathReceiver, ICostmapReceiver, IRobotStateReceiver
)


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class ROS2MapReceiver(IMapReceiver):
    """ROS2 implementation of map receiver."""
    
    def __init__(self, node: Node, topic: str = "/map"):
        self._node = node
        self._callback: Optional[Callable[[MapData], None]] = None
        self._latest: Optional[MapData] = None
        self._lock = threading.Lock()
        
        # QoS profile for map topic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._subscription = node.create_subscription(
            OccupancyGrid,
            topic,
            self._on_message,
            qos
        )
        node.get_logger().info(f"Subscribed to {topic}")
    
    def _on_message(self, msg: OccupancyGrid) -> None:
        """Convert ROS2 message to domain entity."""
        map_data = MapData(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin=Pose2D(
                x=msg.info.origin.position.x,
                y=msg.info.origin.position.y,
                theta=euler_from_quaternion(
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w
                )[2]
            ),
            data=np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )
        )
        
        with self._lock:
            self._latest = map_data
        
        if self._callback:
            self._callback(map_data)
    
    def set_callback(self, callback: Callable[[MapData], None]) -> None:
        self._callback = callback
    
    def get_latest(self) -> Optional[MapData]:
        with self._lock:
            return self._latest


class ROS2LaserReceiver(ILaserReceiver):
    """ROS2 implementation of laser scan receiver."""
    
    def __init__(self, node: Node, topic: str = "/scan"):
        self._node = node
        self._callback: Optional[Callable[[LaserScanData], None]] = None
        self._latest: Optional[LaserScanData] = None
        self._lock = threading.Lock()
        
        # Use simple QoS depth - ROS2 will match best available
        self._subscription = node.create_subscription(
            LaserScan,
            topic,
            self._on_message,
            10  # Queue depth with default QoS
        )
        node.get_logger().info(f"Subscribed to {topic}")
    
    def _on_message(self, msg: LaserScan) -> None:
        scan_data = LaserScanData(
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=np.array(msg.ranges, dtype=np.float32),
            intensities=np.array(msg.intensities, dtype=np.float32) if msg.intensities else None
        )
        
        with self._lock:
            self._latest = scan_data
        
        if self._callback:
            self._callback(scan_data)
    
    def set_callback(self, callback: Callable[[LaserScanData], None]) -> None:
        self._callback = callback
    
    def get_latest(self) -> Optional[LaserScanData]:
        with self._lock:
            return self._latest


class ROS2PoseReceiver(IPoseReceiver):
    """ROS2 implementation of pose receiver from odometry."""
    
    def __init__(self, node: Node, topic: str = "/odom"):
        self._node = node
        self._callback: Optional[Callable[[Pose2D], None]] = None
        self._latest: Optional[Pose2D] = None
        self._lock = threading.Lock()
        
        # Use simple QoS depth - ROS2 will match best available
        self._subscription = node.create_subscription(
            Odometry,
            topic,
            self._on_message,
            10  # Queue depth with default QoS
        )
        node.get_logger().info(f"Subscribed to {topic}")
    
    def _on_message(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Log occasionally for debugging (every ~1s)
        current_time = time.time()
        if not hasattr(self, '_last_log_time'):
             self._last_log_time = 0
        if current_time - self._last_log_time > 1.0:
             self._node.get_logger().info(f"[DEBUG] Odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, th={yaw:.2f}")
             self._last_log_time = current_time
        
        pose = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            theta=yaw
        )
        
        with self._lock:
            self._latest = pose
        
        if self._callback:
            self._callback(pose)
    
    def set_callback(self, callback: Callable[[Pose2D], None]) -> None:
        self._callback = callback
    
    def get_latest(self) -> Optional[Pose2D]:
        with self._lock:
            return self._latest


class ROS2PathReceiver(IPathReceiver):
    """ROS2 implementation of path receiver."""
    
    def __init__(
        self, 
        node: Node, 
        global_topic: str = "/plan",
        local_topic: str = "/local_plan"
    ):
        self._node = node
        self._global_callback: Optional[Callable[[PathData], None]] = None
        self._local_callback: Optional[Callable[[PathData], None]] = None
        self._global_path: Optional[PathData] = None
        self._local_path: Optional[PathData] = None
        self._lock = threading.Lock()
        
        # Use simple QoS depth for compatibility
        self._global_sub = node.create_subscription(
            Path,
            global_topic,
            self._on_global_path,
            10
        )
        
        self._local_sub = node.create_subscription(
            Path,
            local_topic,
            self._on_local_path,
            10
        )
        node.get_logger().info(f"Subscribed to {global_topic} and {local_topic}")
    
    def _convert_path(self, msg: Path) -> PathData:
        poses = []
        for pose_stamped in msg.poses:
            _, _, yaw = euler_from_quaternion(
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            )
            poses.append(Pose2D(
                x=pose_stamped.pose.position.x,
                y=pose_stamped.pose.position.y,
                theta=yaw
            ))
        return PathData(poses=poses, frame_id=msg.header.frame_id)
    
    def _on_global_path(self, msg: Path) -> None:
        path_data = self._convert_path(msg)
        with self._lock:
            self._global_path = path_data
        if self._global_callback:
            self._global_callback(path_data)
    
    def _on_local_path(self, msg: Path) -> None:
        path_data = self._convert_path(msg)
        with self._lock:
            self._local_path = path_data
        if self._local_callback:
            self._local_callback(path_data)
    
    def set_global_path_callback(self, callback: Callable[[PathData], None]) -> None:
        self._global_callback = callback
    
    def set_local_path_callback(self, callback: Callable[[PathData], None]) -> None:
        self._local_callback = callback
    
    def get_global_path(self) -> Optional[PathData]:
        with self._lock:
            return self._global_path
    
    def get_local_path(self) -> Optional[PathData]:
        with self._lock:
            return self._local_path


class ROS2CostmapReceiver(ICostmapReceiver):
    """ROS2 implementation of costmap receiver."""
    
    def __init__(
        self, 
        node: Node,
        global_topic: str = "/global_costmap/costmap",
        local_topic: str = "/local_costmap/costmap"
    ):
        self._node = node
        self._global_callback: Optional[Callable[[CostmapData], None]] = None
        self._local_callback: Optional[Callable[[CostmapData], None]] = None
        self._global_costmap: Optional[CostmapData] = None
        self._local_costmap: Optional[CostmapData] = None
        self._lock = threading.Lock()
        
        # Use simple QoS depth for compatibility
        self._global_sub = node.create_subscription(
            OccupancyGrid,
            global_topic,
            self._on_global_costmap,
            10
        )
        
        self._local_sub = node.create_subscription(
            OccupancyGrid,
            local_topic,
            self._on_local_costmap,
            10
        )
        node.get_logger().info(f"Subscribed to costmaps")
    
    def _convert_costmap(self, msg: OccupancyGrid) -> CostmapData:
        return CostmapData(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin=Pose2D(
                x=msg.info.origin.position.x,
                y=msg.info.origin.position.y,
                theta=0.0
            ),
            data=np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )
        )
    
    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        costmap = self._convert_costmap(msg)
        with self._lock:
            self._global_costmap = costmap
        if self._global_callback:
            self._global_callback(costmap)
    
    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        costmap = self._convert_costmap(msg)
        with self._lock:
            self._local_costmap = costmap
        if self._local_callback:
            self._local_callback(costmap)
    
    def set_global_costmap_callback(self, callback: Callable[[CostmapData], None]) -> None:
        self._global_callback = callback
    
    def set_local_costmap_callback(self, callback: Callable[[CostmapData], None]) -> None:
        self._local_callback = callback
    
    def get_global_costmap(self) -> Optional[CostmapData]:
        with self._lock:
            return self._global_costmap
    
    def get_local_costmap(self) -> Optional[CostmapData]:
        with self._lock:
            return self._local_costmap
