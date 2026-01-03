"""
Domain Entities for Navigation Viewer

Core domain objects representing navigation data. These are pure Python
dataclasses with no dependencies on ROS2 or UI frameworks.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np
from enum import Enum
import uuid


class NavigationStatus(Enum):
    """Current navigation status."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass
class Pose2D:
    """2D pose representing position and orientation."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # Radians
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)


@dataclass
class MapData:
    """
    Occupancy grid map data.
    
    Values: 0 = free, 100 = occupied, -1 = unknown
    """
    width: int = 0
    height: int = 0
    resolution: float = 0.05  # meters per cell
    origin: Pose2D = field(default_factory=Pose2D)
    data: Optional[np.ndarray] = None  # 2D numpy array
    
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to map cell indices."""
        if self.resolution == 0:
            return (0, 0)
        map_x = int((world_x - self.origin.x) / self.resolution)
        map_y = int((world_y - self.origin.y) / self.resolution)
        return (map_x, map_y)
    
    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """Convert map cell indices to world coordinates."""
        world_x = map_x * self.resolution + self.origin.x
        world_y = map_y * self.resolution + self.origin.y
        return (world_x, world_y)


@dataclass
class LaserScanData:
    """
    Laser scan data from LiDAR sensor.
    """
    angle_min: float = 0.0  # Radians
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 12.0
    ranges: Optional[np.ndarray] = None  # 1D array of range values
    intensities: Optional[np.ndarray] = None
    
    def get_cartesian_points(self) -> List[Tuple[float, float]]:
        """Convert polar ranges to Cartesian coordinates in sensor frame."""
        if self.ranges is None:
            return []
        
        points = []
        angle = self.angle_min
        for r in self.ranges:
            if self.range_min <= r <= self.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
            angle += self.angle_increment
        return points


@dataclass
class PathData:
    """Navigation path as a list of poses."""
    poses: List[Pose2D] = field(default_factory=list)
    frame_id: str = "map"
    
    def __len__(self) -> int:
        return len(self.poses)
    
    def is_empty(self) -> bool:
        return len(self.poses) == 0


@dataclass
class CostmapData:
    """
    Costmap data for visualization.
    
    Values: 0 = free, 254 = lethal obstacle, 253 = inscribed, etc.
    """
    width: int = 0
    height: int = 0
    resolution: float = 0.05
    origin: Pose2D = field(default_factory=Pose2D)
    data: Optional[np.ndarray] = None


@dataclass
class NavigationGoal:
    """Goal pose for navigation."""
    pose: Pose2D = field(default_factory=Pose2D)
    frame_id: str = "map"


@dataclass
class RobotState:
    """Complete robot state for visualization."""
    pose: Pose2D = field(default_factory=Pose2D)
    velocity_linear: float = 0.0
    velocity_angular: float = 0.0
    navigation_status: NavigationStatus = NavigationStatus.IDLE
    battery_level: float = 100.0  # Percentage


@dataclass
class Station:
    """Named waypoint/station for AMR."""
    name: str
    pose: Pose2D
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    color: str = "#00ff00"  # Hex color for visualization


@dataclass
class RestrictedZone:
    """Polygon zone where navigation is restricted."""
    name: str
    points: List[Tuple[float, float]]  # List of (x, y) points forming polygon
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    is_active: bool = True


class LoopType(Enum):
    """Type of route looping."""
    ONCE = "once"
    INFINITE = "infinite"
    COUNT = "count"


@dataclass
class Route:
    """Defined path visiting multiple stations."""
    name: str
    waypoints: List[str] = field(default_factory=list)  # List of Station UUIDs
    loop_type: str = "once"  # serialized string of LoopType
    loop_count: int = 1
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    
    @property
    def loop_type_enum(self) -> LoopType:
        try:
            return LoopType(self.loop_type)
        except ValueError:
            return LoopType.ONCE
