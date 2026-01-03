"""
Application Coordinator

Dependency injection and wiring of all components.
Manages ROS2 node lifecycle and connects adapters to UI.
"""

import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtCore import QObject, pyqtSignal, QTimer

from ..domain.entities import (
    MapData, LaserScanData, Pose2D, PathData, 
    CostmapData, NavigationGoal, Station, RestrictedZone
)
from .station_manager import StationManager
from .zone_manager import ZoneManager
from .route_manager import RouteManager
from .route_executor import RouteExecutor
from PyQt5.QtWidgets import QMessageBox
import math
from ..adapters.ros2_subscribers import (
    ROS2MapReceiver, ROS2LaserReceiver, ROS2PoseReceiver,
    ROS2PathReceiver, ROS2CostmapReceiver
)
from ..adapters.ros2_publishers import (
    ROS2GoalSender, ROS2PoseSetter, ROS2TeleopController, ROS2ZoneMaskPublisher
)
from ..adapters.map_adapter import MapAdapter
from ..adapters.service_adapter import NavigationServiceAdapter
from ..infrastructure.main_window import MainWindow


class SignalBridge(QObject):
    """
    Bridge between ROS2 callbacks (background thread) and Qt signals (main thread).
    Qt signals are thread-safe and can be used to update UI from background threads.
    """
    
    map_received = pyqtSignal(object)
    laser_received = pyqtSignal(object)
    pose_received = pyqtSignal(object)
    global_path_received = pyqtSignal(object)
    local_path_received = pyqtSignal(object)
    global_costmap_received = pyqtSignal(object)
    local_costmap_received = pyqtSignal(object)


class ApplicationCoordinator:
    """
    Coordinates all application components using dependency injection.
    """
    
    def __init__(self, window: MainWindow, is_local: bool = False):
        self._window = window
        self._is_local = is_local
        self._node: Optional[Node] = None
        self._spin_timer: Optional[QTimer] = None
        self._running = False
        
        # Signal bridge for thread-safe UI updates
        self._signals = SignalBridge()
        
        # Adapters (initialized in start())
        self._map_receiver: Optional[ROS2MapReceiver] = None
        self._laser_receiver: Optional[ROS2LaserReceiver] = None
        self._pose_receiver: Optional[ROS2PoseReceiver] = None
        self._path_receiver: Optional[ROS2PathReceiver] = None
        self._costmap_receiver: Optional[ROS2CostmapReceiver] = None
        self._goal_sender: Optional[ROS2GoalSender] = None
        self._pose_setter: Optional[ROS2PoseSetter] = None
        
        # AMR Managers
        self._station_manager = StationManager()
        self._zone_manager = ZoneManager()
        self._route_manager = RouteManager()
        self._route_executor = RouteExecutor(self._station_manager)
        
        # Simple Goal Monitoring
        self._current_nav_goal: Optional[NavigationGoal] = None
        
        self._setup_signal_connections()
    
    def _setup_signal_connections(self) -> None:
        """Connect signals to window update methods."""
        self._signals.map_received.connect(self._window.update_map)
        self._signals.laser_received.connect(self._window.update_laser)
        self._signals.pose_received.connect(self._window.update_robot_pose)
        self._signals.global_path_received.connect(self._window.update_global_path)
        self._signals.local_path_received.connect(self._window.update_local_path)
        self._signals.global_costmap_received.connect(self._window.update_global_costmap)
        self._signals.local_costmap_received.connect(self._window.update_local_costmap)
    
    def start(self, node: Node = None) -> bool:
        """Initialize ROS2 and start spinning.
        
        Args:
            node: Pre-created ROS2 node (required for proper Qt integration)
        """
        try:
            if node is None:
                # Fallback: create node if not provided
                if not rclpy.ok():
                    rclpy.init()
                self._node = Node('nav_viewer_app')
            else:
                self._node = node
            
            self._node.get_logger().info("Navigation Viewer started")
            
            # Create adapters
            self._create_adapters()
            
            # Wire callbacks
            self._wire_callbacks()
            
            # Set up window callbacks
            self._window.set_goal_callback(self._on_goal_requested)
            self._window.set_initial_pose_callback(self._on_initial_pose_requested)
            self._window.set_cancel_callback(self._on_cancel_requested)
            self._window.set_teleop_callback(self._on_teleop_requested)
            
            # Wire AMR Widgets
            self._wire_amr_widgets()
            
            # Initial refresh
            self._refresh_amr_widgets()
            
            # Use QTimer to spin ROS2 on the main thread
            # This is more reliable than background thread with Qt
            self._spin_timer = QTimer()
            self._spin_timer.timeout.connect(self._spin_once)
            self._spin_timer.start(10)  # 100Hz polling
            
            self._running = True
            self._node.get_logger().info("ROS2 spin timer started (10ms interval)")
            
            return True
            
        except Exception as e:
            print(f"Failed to start ROS2: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _create_adapters(self) -> None:
        """Create all ROS2 adapters."""
        # Input adapters (subscribers)
        self._map_receiver = ROS2MapReceiver(self._node, "/map")
        self._laser_receiver = ROS2LaserReceiver(self._node, "/scan")
        self._pose_receiver = ROS2PoseReceiver(self._node, "/odom")
        self._path_receiver = ROS2PathReceiver(
            self._node, 
            global_topic="/plan",
            local_topic="/local_plan"
        )
        self._costmap_receiver = ROS2CostmapReceiver(
            self._node,
            global_topic="/global_costmap/costmap",
            local_topic="/local_costmap/costmap"
        )
        
        # Output adapters (publishers)
        self._goal_sender = ROS2GoalSender(self._node, "/goal_pose")
        self._pose_setter = ROS2PoseSetter(self._node, "/initialpose")
        if self._is_local:
            # Reverting to direct /cmd_vel as analysis showed it's silent (no conflict)
            # and routing through smoother (/cmd_vel_nav) caused issues.
            self._teleop_controller = ROS2TeleopController(self._node, "/cmd_vel")
        else:
            self._teleop_controller = ROS2TeleopController(self._node, "/cmd_vel")
            
        # Map I/O Adapter
        self._map_adapter = MapAdapter(self._node)
        
        # Zone Mask Publisher
        self._zone_mask_publisher = ROS2ZoneMaskPublisher(self._node)
        
        # Service Adapter (Recovery)
        self._service_adapter = NavigationServiceAdapter(self._node)
    
    def _wire_callbacks(self) -> None:
        """Wire adapter callbacks to signal emitters."""
        # Map I/O
        self._window.save_map_requested.connect(self._map_adapter.save_map)
        self._window.load_map_requested.connect(self._map_adapter.load_map)
        
        self._map_adapter.save_finished.connect(self._on_map_save_finished)
        self._map_adapter.load_finished.connect(self._on_map_load_finished)

        # Recovery Services
        self._window.control_panel.clear_costmaps_clicked.connect(self._service_adapter.clear_costmaps)
        self._service_adapter.action_finished.connect(self._on_service_action_finished)

        # Map

        # Map
        self._map_receiver.set_callback(
            lambda data: self._signals.map_received.emit(data)
        )
        
        # Laser
        self._laser_receiver.set_callback(
            lambda data: self._signals.laser_received.emit(data)
        )
        
        # Pose
        self._pose_receiver.set_callback(
            lambda data: [
                self._signals.pose_received.emit(data),
                self._check_navigation_status(data)
            ]
        )
        
        # Paths
        self._path_receiver.set_global_path_callback(
            lambda data: self._signals.global_path_received.emit(data)
        )
        self._path_receiver.set_local_path_callback(
            lambda data: self._signals.local_path_received.emit(data)
        )
        
        # Costmaps
        self._costmap_receiver.set_global_costmap_callback(
            lambda data: self._signals.global_costmap_received.emit(data)
        )
        self._costmap_receiver.set_local_costmap_callback(
            lambda data: self._signals.local_costmap_received.emit(data)
        )
        
        # Internal Signals
        self._zone_manager.zones_changed.connect(self._publish_zone_mask)
        self._signals.map_received.connect(self._on_map_received)
        
        self._node.get_logger().info("All callbacks wired")
    
    def _on_map_received(self, map_data: MapData) -> None:
        """Handle new map data for zone mask alignment."""
        # Republish zone mask to match new map
        self._publish_zone_mask()
        
    def _publish_zone_mask(self) -> None:
        """Publish updated zone mask aligned with current map."""
        map_data = self._map_receiver.get_latest()
        zones = self._zone_manager.get_zones()
        if map_data and zones:
            self._zone_mask_publisher.publish_mask(zones, map_data)
    
    def _spin_once(self) -> None:
        """Called by QTimer to spin ROS2 on main thread."""
        if self._node and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0)
    
    def _on_goal_requested(self, goal: NavigationGoal) -> None:
        """Handle goal request from UI."""
        # Check restricted zones
        if self._zone_manager.is_point_restricted(goal.pose.x, goal.pose.y):
            QMessageBox.warning(
                self._window, 
                "Restricted Zone", 
                "Cannot navigate to this location: It is inside a Restricted Zone!"
            )
            return

        if self._goal_sender:
            self._goal_sender.send_goal(goal)
    
    def _on_initial_pose_requested(self, pose: Pose2D) -> None:
        """Handle initial pose request from UI."""
        if self._pose_setter:
            self._pose_setter.set_initial_pose(pose)
    
    def _on_cancel_requested(self) -> None:
        """Handle navigation cancel request from UI."""
        if self._goal_sender:
            self._goal_sender.cancel_goal()
            
    def _on_teleop_requested(self, linear: float, angular: float) -> None:
        """Handle teleop request from UI."""
        if self._teleop_controller:
            self._teleop_controller.send_velocity(linear, angular)
    
    def stop(self) -> None:
        """Stop ROS2 and clean up."""
        self._running = False
        
        if hasattr(self, '_spin_timer') and self._spin_timer:
            self._spin_timer.stop()
        
        if self._node:
            self._node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

    def _wire_amr_widgets(self) -> None:
        """Connect signals for AMR widgets."""
        ctrl = self._window.control_panel
        
        # Stations
        ctrl.station_list.save_station_requested.connect(self._on_save_station)
        ctrl.station_list.goto_station_requested.connect(self._on_goto_station)
        ctrl.station_list.delete_station_requested.connect(self._on_delete_station)
        
        # Zones
        # Zones
        ctrl.zone_list.toggle_draw_mode.connect(self._window.map_widget.set_drawing_mode)
        ctrl.zone_list.delete_zone_requested.connect(self._on_delete_zone)
        
        # Connect Map interactions
        self._window.map_widget.polygon_completed.connect(self._on_polygon_completed)
        self._window.map_widget.station_created_at_point.connect(self._on_station_create_requested)
        
        # Routes
        ctrl.route_list.set_managers(self._route_manager, self._station_manager)
        ctrl.route_list.run_route_requested.connect(self._on_run_route)
        ctrl.route_list.stop_route_requested.connect(self._on_stop_route)
        
        self._route_executor.goal_requested.connect(self._on_goal_requested_by_route)
        self._route_executor.started.connect(lambda n: ctrl.route_list.set_running_state(True))
        self._route_executor.stopped.connect(lambda: ctrl.route_list.set_running_state(False))
        self._route_executor.finished.connect(lambda: ctrl.route_list.set_running_state(False))
        self._route_executor.status_changed.connect(ctrl.set_navigation_status)

    def _refresh_amr_widgets(self) -> None:
        """Push latest data to UI."""
        stations = self._station_manager.get_stations()
        zones = self._zone_manager.get_zones()
        
        self._window.control_panel.station_list.update_list(stations)
        self._window.control_panel.zone_list.update_list(zones)
        self._window.control_panel.route_list.refresh_list()
        
        self._window.map_widget.set_stations(stations)
        self._window.map_widget.set_zones(zones)

    # === AMR Callbacks ===

    def _on_save_station(self, name: str) -> None:
        if self._window.map_widget.canvas._robot_pose:
            pose = self._window.map_widget.canvas._robot_pose
            # Clone pose to avoid reference issues
            saved_pose = Pose2D(x=pose.x, y=pose.y, theta=pose.theta)
            station = Station(name=name, pose=saved_pose)
            self._station_manager.add_station(station)
            self._refresh_amr_widgets()

    def _on_goto_station(self, pose: Pose2D) -> None:
        goal = NavigationGoal(pose=pose)
        self._on_goal_requested(goal)

    def _on_delete_station(self, sid: str) -> None:
        self._station_manager.remove_station(sid)
        self._refresh_amr_widgets()

    def _on_delete_zone(self, zid: str) -> None:
        self._zone_manager.remove_zone(zid)
        self._refresh_amr_widgets()

    def _on_polygon_completed(self, points: list) -> None:
        # Convert QPointF list to [(x,y)] tuples
        poly_points = [(p.x(), p.y()) for p in points]
        
        # Create zone
        # Prompt for name
        from PyQt5.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self._window, "New Zone", "Zone Name:")
        if ok and name:
            zone = RestrictedZone(name=name, points=poly_points)
            self._zone_manager.add_zone(zone)
            self._refresh_amr_widgets()

    def _on_station_create_requested(self, point) -> None:
        """Handle station creation from map double click."""
        from PyQt5.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self._window, "New Station", "Station Name:")
        if ok and name:
            # Create station at clicked point with 0 orientation
            pose = Pose2D(x=point.x(), y=point.y(), theta=0.0)
            station = Station(name=name, pose=pose)
            self._station_manager.add_station(station)
            self._refresh_amr_widgets()

    def _on_map_save_finished(self, success: bool, message: str) -> None:
        if success:
            QMessageBox.information(self._window, "Map Saved", message)
        else:
            QMessageBox.critical(self._window, "Save Failed", message)

    def _on_map_load_finished(self, success: bool, message: str) -> None:
        if success:
            QMessageBox.information(self._window, "Map Loaded", message)
            # Trigger map refresh (though load_map service might trigger /map update automatically depending on node config)
        else:
            QMessageBox.critical(self._window, "Load Failed", message)

    def _on_service_action_finished(self, success: bool, message: str) -> None:
        if success:
             QMessageBox.information(self._window, "Success", message)
        else:
             QMessageBox.warning(self._window, "Action Failed", message)

    # === Route Logic ===

    def _on_run_route(self, route_id: str) -> None:
        """Start route execution."""
        route = self._route_manager.get_route_by_id(route_id)
        if route:
            # Cancel current goal first
            if self._goal_sender:
                self._goal_sender.cancel_goal()
            self._route_executor.start_route(route)
            
    def _on_stop_route(self) -> None:
        """Stop route execution."""
        self._route_executor.stop()
        if self._goal_sender:
            self._goal_sender.cancel_goal()
        self._current_nav_goal = None
        
    def _on_goal_requested_by_route(self, goal: NavigationGoal) -> None:
        """Handle goal from route executor."""
        # Use existing handler but ensure we track it
        self._on_goal_requested(goal)
    
    def _check_navigation_status(self, current_pose: Pose2D) -> None:
        """Check if robot reached the goal."""
        if self._current_nav_goal is None:
            return
            
        dx = current_pose.x - self._current_nav_goal.pose.x
        dy = current_pose.y - self._current_nav_goal.pose.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Thresholds: 0.3m position
        if dist < 0.3:
            # We assume orientation is handled by Nav2 final rotation
            # For strict checking we could check theta too
            from ..domain.entities import NavigationStatus
            self._current_nav_goal = None
            self._route_executor.on_navigation_status(NavigationStatus.SUCCEEDED)
