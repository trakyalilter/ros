"""
Output Ports

Abstract interfaces for sending commands to external systems.
These define what commands the application can send.
"""

from abc import ABC, abstractmethod
from typing import Optional, List
from ..domain.entities import Pose2D, NavigationGoal


class IGoalSender(ABC):
    """Interface for sending navigation goals."""
    
    @abstractmethod
    def send_goal(self, goal: NavigationGoal) -> bool:
        """
        Send a navigation goal.
        
        Args:
            goal: The navigation goal to send
            
        Returns:
            True if the goal was accepted, False otherwise
        """
        pass
    
    @abstractmethod
    def cancel_goal(self) -> bool:
        """
        Cancel the current navigation goal.
        
        Returns:
            True if cancellation was successful
        """
        pass
    
    @abstractmethod
    def is_navigation_active(self) -> bool:
        """Check if there's an active navigation goal."""
        pass


class IPoseSetter(ABC):
    """Interface for setting initial pose estimate."""
    
    @abstractmethod
    def set_initial_pose(
        self, 
        pose: Pose2D, 
        covariance_x: float = 0.25,
        covariance_y: float = 0.25,
        covariance_yaw: float = 0.07
    ) -> None:
        """
        Set the initial pose estimate for localization.
        
        Args:
            pose: The initial pose estimate
            covariance_x: Uncertainty in x position
            covariance_y: Uncertainty in y position
            covariance_yaw: Uncertainty in orientation
        """
        pass


class IWaypointSender(ABC):
    """Interface for sending waypoint sequences."""
    
    @abstractmethod
    def send_waypoints(self, waypoints: List[NavigationGoal]) -> bool:
        """
        Send a sequence of waypoints to follow.
        
        Args:
            waypoints: List of navigation goals to follow in order
            
        Returns:
            True if waypoints were accepted
        """
        pass
    
    @abstractmethod
    def cancel_waypoints(self) -> bool:
        """Cancel the current waypoint sequence."""
        pass


class ITeleopController(ABC):
    """Interface for teleop control (optional feature)."""
    
    @abstractmethod
    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        """
        Send velocity command for manual control.
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
        """
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Send stop command."""
        pass
