"""
Input Ports

Abstract interfaces for receiving data from external sources.
These define what data the application expects to receive.
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional
from ..domain.entities import (
    MapData, LaserScanData, Pose2D, PathData, 
    CostmapData, RobotState
)


class IMapReceiver(ABC):
    """Interface for receiving map data."""
    
    @abstractmethod
    def set_callback(self, callback: Callable[[MapData], None]) -> None:
        """Set callback to be invoked when map data is received."""
        pass
    
    @abstractmethod
    def get_latest(self) -> Optional[MapData]:
        """Get the latest received map data."""
        pass


class ILaserReceiver(ABC):
    """Interface for receiving laser scan data."""
    
    @abstractmethod
    def set_callback(self, callback: Callable[[LaserScanData], None]) -> None:
        """Set callback to be invoked when laser scan is received."""
        pass
    
    @abstractmethod
    def get_latest(self) -> Optional[LaserScanData]:
        """Get the latest received laser scan."""
        pass


class IPoseReceiver(ABC):
    """Interface for receiving robot pose (odometry) data."""
    
    @abstractmethod
    def set_callback(self, callback: Callable[[Pose2D], None]) -> None:
        """Set callback to be invoked when pose is received."""
        pass
    
    @abstractmethod
    def get_latest(self) -> Optional[Pose2D]:
        """Get the latest received pose."""
        pass


class IPathReceiver(ABC):
    """Interface for receiving navigation path data."""
    
    @abstractmethod
    def set_global_path_callback(self, callback: Callable[[PathData], None]) -> None:
        """Set callback for global path updates."""
        pass
    
    @abstractmethod
    def set_local_path_callback(self, callback: Callable[[PathData], None]) -> None:
        """Set callback for local path updates."""
        pass
    
    @abstractmethod
    def get_global_path(self) -> Optional[PathData]:
        """Get the latest global path."""
        pass
    
    @abstractmethod
    def get_local_path(self) -> Optional[PathData]:
        """Get the latest local path."""
        pass


class ICostmapReceiver(ABC):
    """Interface for receiving costmap data."""
    
    @abstractmethod
    def set_global_costmap_callback(self, callback: Callable[[CostmapData], None]) -> None:
        """Set callback for global costmap updates."""
        pass
    
    @abstractmethod
    def set_local_costmap_callback(self, callback: Callable[[CostmapData], None]) -> None:
        """Set callback for local costmap updates."""
        pass
    
    @abstractmethod
    def get_global_costmap(self) -> Optional[CostmapData]:
        """Get the latest global costmap."""
        pass
    
    @abstractmethod
    def get_local_costmap(self) -> Optional[CostmapData]:
        """Get the latest local costmap."""
        pass


class IRobotStateReceiver(ABC):
    """Interface for receiving complete robot state."""
    
    @abstractmethod
    def set_callback(self, callback: Callable[[RobotState], None]) -> None:
        """Set callback for robot state updates."""
        pass
    
    @abstractmethod
    def get_latest(self) -> Optional[RobotState]:
        """Get the latest robot state."""
        pass
