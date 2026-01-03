"""
Domain Services

Abstract service interfaces for the domain layer.
These define the contracts that adapters must implement.
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional
from .entities import (
    MapData, LaserScanData, Pose2D, PathData, 
    CostmapData, NavigationGoal, RobotState
)


class NavigationService(ABC):
    """Abstract service for navigation operations."""
    
    @abstractmethod
    def send_goal(self, goal: NavigationGoal) -> bool:
        """Send a navigation goal. Returns True if accepted."""
        pass
    
    @abstractmethod
    def cancel_navigation(self) -> bool:
        """Cancel current navigation. Returns True if successful."""
        pass
    
    @abstractmethod
    def set_initial_pose(self, pose: Pose2D, covariance: Optional[list] = None) -> None:
        """Set the initial pose estimate for localization."""
        pass


class DataTransformer(ABC):
    """Abstract service for coordinate transformations."""
    
    @abstractmethod
    def transform_pose(self, pose: Pose2D, from_frame: str, to_frame: str) -> Pose2D:
        """Transform a pose from one frame to another."""
        pass
    
    @abstractmethod
    def transform_laser_to_map(
        self, 
        scan: LaserScanData, 
        robot_pose: Pose2D
    ) -> list:
        """Transform laser scan points to map frame."""
        pass
