
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from typing import Optional, List
from ..domain.entities import Route, Station, LoopType, NavigationStatus, NavigationGoal
from .station_manager import StationManager

class RouteExecutor(QObject):
    """
    Executes a Route by feeding goals to the Navigation system.
    Handles looping and sequencing.
    """
    
    # Signals
    started = pyqtSignal(str) # route name
    stopped = pyqtSignal()
    finished = pyqtSignal()
    waypoint_reached = pyqtSignal(str, int) # station_name, index
    goal_requested = pyqtSignal(NavigationGoal)
    status_changed = pyqtSignal(str) # "Running: Waypoint 1/5", "Idle", etc.
    
    def __init__(self, station_manager: StationManager):
        super().__init__()
        self._station_manager = station_manager
        
        self._active_route: Optional[Route] = None
        self._current_index = 0
        self._current_loop = 0
        self._running = False
        self._paused = False
        
        self._nav_status = NavigationStatus.IDLE
        
    def start_route(self, route: Route):
        """Start executing a route."""
        if not route.waypoints:
             self.status_changed.emit("Empty route")
             return
             
        self._active_route = route
        self._current_index = 0
        self._current_loop = 0
        self._running = True
        self._paused = False
        
        self.started.emit(route.name)
        self._send_next_goal()
        
    def stop(self):
        """Stop execution."""
        self._running = False
        self._active_route = None
        self.stopped.emit()
        self.status_changed.emit("Stopped")
        
    def on_navigation_status(self, status: NavigationStatus):
        """Receive updates from navigation stack."""
        self._nav_status = status
        
        if not self._running or self._paused:
            return
            
        if status == NavigationStatus.SUCCEEDED:
            # Reached current waypoint
            self._handle_waypoint_reached()
        elif status == NavigationStatus.FAILED:
            # Handle failure (abort or retry?) - For now, we stop.
            self.status_changed.emit(f"Failed at waypoint {self._current_index + 1}")
            self.stop()
            
    def _handle_waypoint_reached(self):
        # Notify
        station_id = self._active_route.waypoints[self._current_index]
        station = self._station_manager.get_station_by_id(station_id)
        name = station.name if station else "Unknown"
        self.waypoint_reached.emit(name, self._current_index)
        
        # Increment
        self._current_index += 1
        
        # Check if route complete
        if self._current_index >= len(self._active_route.waypoints):
            self._handle_route_completion()
        else:
            self._send_next_goal()
            
    def _handle_route_completion(self):
        # Check looping
        loop_type = self._active_route.loop_type_enum
        
        should_loop = False
        if loop_type == LoopType.INFINITE:
            should_loop = True
        elif loop_type == LoopType.COUNT:
             self._current_loop += 1
             if self._current_loop < self._active_route.loop_count:
                 should_loop = True
                 
        if should_loop:
            self._current_index = 0
            self.status_changed.emit(f"Starting Loop {self._current_loop + 1}")
            self._send_next_goal()
        else:
            self.finished.emit()
            self.status_changed.emit("Route Finished")
            self._running = False
            
    def _send_next_goal(self):
        if not self._running:
            return

        station_id = self._active_route.waypoints[self._current_index]
        station = self._station_manager.get_station_by_id(station_id)
        
        if not station:
            self.status_changed.emit(f"Station {station_id} not found, aborting.")
            self.stop()
            return
            
        goal = NavigationGoal(pose=station.pose)
        self.goal_requested.emit(goal)
        
        total = len(self._active_route.waypoints)
        self.status_changed.emit(f"Going to {station.name} ({self._current_index + 1}/{total})")
