"""
Main Window

The primary application window assembling all widgets.
"""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QSplitter, QStatusBar, QMenuBar, QMenu, QAction,
    QMessageBox, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QKeySequence

from .theme import Colors
from .widgets.map_widget import MapWidget
from .widgets.control_panel import ControlPanel
from ..domain.entities import (
    MapData, LaserScanData, Pose2D, PathData, 
    CostmapData, NavigationGoal, NavigationStatus
)


class MainWindow(QMainWindow):
    """
    Main application window for Navigation Viewer.
    """
    
    save_map_requested = pyqtSignal(str)
    load_map_requested = pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setWindowTitle("ROS2 Navigation Viewer")
        self.setMinimumSize(1200, 800)
        self.setFocusPolicy(Qt.StrongFocus)
        
        self._setup_ui()
        self._setup_menu()
        self._setup_status_bar()
        self._connect_signals()
        
        # Goal callback (set by coordinator)
        self._goal_callback = None
        self._initial_pose_callback = None
        self._cancel_callback = None
        self._teleop_callback = None
        
        # Teleop state
        self._pressed_keys = set()
        self._current_vx = 0.0
        self._current_wz = 0.0
        self._teleop_timer = QTimer()
        self._teleop_timer.setInterval(50)  # 20 Hz for smoother control
        self._teleop_timer.timeout.connect(self._process_teleop_keys)
        self._teleop_timer.start()
    
    def _setup_ui(self) -> None:
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)
        
        # Map widget (main view)
        self.map_widget = MapWidget()
        splitter.addWidget(self.map_widget)
        
        # Control panel (side)
        self.control_panel = ControlPanel()
        splitter.addWidget(self.control_panel)
        
        # Set initial sizes (70% map, 30% controls)
        splitter.setSizes([700, 300])
        
        layout.addWidget(splitter)
    
    def _setup_menu(self) -> None:
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("&File")
        
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        file_menu.addSeparator()
        
        save_map_action = QAction("&Save Map...", self)
        save_map_action.triggered.connect(self._on_save_map)
        file_menu.addAction(save_map_action)
        
        load_map_action = QAction("&Load Map...", self)
        load_map_action.triggered.connect(self._on_load_map)
        file_menu.addAction(load_map_action)
        
        # View menu
        view_menu = menubar.addMenu("&View")
        
        center_action = QAction("&Center on Robot", self)
        center_action.setShortcut(QKeySequence("Ctrl+C"))
        center_action.triggered.connect(self.map_widget.canvas.center_on_robot)
        view_menu.addAction(center_action)
        
        reset_view_action = QAction("&Reset View", self)
        reset_view_action.setShortcut(QKeySequence("Home"))
        reset_view_action.triggered.connect(self._reset_view)
        view_menu.addAction(reset_view_action)
        
        view_menu.addSeparator()
        
        # Toggle actions
        self.toggle_laser_action = QAction("Show &Laser Scan", self, checkable=True)
        self.toggle_laser_action.setChecked(True)
        self.toggle_laser_action.triggered.connect(
            lambda v: setattr(self.map_widget.canvas, 'show_laser', v) or self.map_widget.canvas.update()
        )
        view_menu.addAction(self.toggle_laser_action)
        
        self.toggle_global_path_action = QAction("Show &Global Path", self, checkable=True)
        self.toggle_global_path_action.setChecked(True)
        self.toggle_global_path_action.triggered.connect(
            lambda v: setattr(self.map_widget.canvas, 'show_global_path', v) or self.map_widget.canvas.update()
        )
        view_menu.addAction(self.toggle_global_path_action)
        
        self.toggle_local_path_action = QAction("Show L&ocal Path", self, checkable=True)
        self.toggle_local_path_action.setChecked(True)
        self.toggle_local_path_action.triggered.connect(
            lambda v: setattr(self.map_widget.canvas, 'show_local_path', v) or self.map_widget.canvas.update()
        )
        view_menu.addAction(self.toggle_local_path_action)
        
        # Navigation menu
        nav_menu = menubar.addMenu("&Navigation")
        
        cancel_action = QAction("&Cancel Navigation", self)
        cancel_action.setShortcut(QKeySequence("Escape"))
        cancel_action.triggered.connect(self._on_cancel_navigation)
        nav_menu.addAction(cancel_action)
        
        # Help menu
        help_menu = menubar.addMenu("&Help")
        
        about_action = QAction("&About", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)
    
    def _setup_status_bar(self) -> None:
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        self.status_bar.showMessage("Ready - Right-click on map to set navigation goal")
    
    def _connect_signals(self) -> None:
        # Map widget signals
        self.map_widget.goal_requested.connect(self._on_goal_requested)
        self.map_widget.initial_pose_requested.connect(self._on_initial_pose_requested)
        self.map_widget.velocity_command.connect(self._on_teleop_command)
        
        # Control panel signals
        self.control_panel.send_goal_clicked.connect(self._on_goal_requested)
        self.control_panel.set_initial_pose_clicked.connect(self._on_initial_pose_requested)
        self.control_panel.cancel_navigation_clicked.connect(self._on_cancel_navigation)
        self.control_panel.send_velocity.connect(self._on_teleop_command)
    
    def _reset_view(self) -> None:
        self.map_widget.canvas._scale = 50.0
        self.map_widget.canvas._offset.setX(0)
        self.map_widget.canvas._offset.setY(0)
        self.map_widget.canvas.update()
    
    def _show_about(self) -> None:
        QMessageBox.about(
            self,
            "About Navigation Viewer",
            "<h3>ROS2 Navigation Viewer</h3>"
            "<p>A PyQt5 application for visualizing ROS2 navigation data.</p>"
            "<p>Features:</p>"
            "<ul>"
            "<li>Map visualization</li>"
            "<li>Laser scan overlay</li>"
            "<li>Navigation path display</li>"
            "<li>Interactive goal setting</li>"
            "<li>Manual Robot Control (WASD)</li>"
            "</ul>"
            "<p style='color: gray;'>Built with hexagonal architecture</p>"
        )
    
    def keyPressEvent(self, event) -> None:
        """Handle keyboard input for teleop."""
        key = event.key()
        
        # Handle teleop keys
        if key in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            if not event.isAutoRepeat():
                self._pressed_keys.add(key)
        elif key == Qt.Key_Space:
            if not event.isAutoRepeat():
                self._pressed_keys.clear()
                self._on_teleop_command(0.0, 0.0)
        else:
            # Propagate other keys (shortcuts, etc.)
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        """Handle keyboard release for teleop."""
        if event.isAutoRepeat():
            return
            
        key = event.key()
        if key in self._pressed_keys:
            self._pressed_keys.remove(key)
            # Timer will handle ramp down
        else:
            super().keyReleaseEvent(event)
    
    def _process_teleop_keys(self) -> None:
        """Calculate and send velocity based on pressed keys with ramping."""
        # Allow running if keys pressed OR if we need to ramp down
        if not self._pressed_keys and self._current_vx == 0.0 and self._current_wz == 0.0:
            return
            
        target_vx = 0.0
        target_wz = 0.0
        
        if Qt.Key_W in self._pressed_keys:
            target_vx += 0.3
        if Qt.Key_S in self._pressed_keys:
            target_vx -= 0.3
        if Qt.Key_A in self._pressed_keys:
            target_wz += 0.8
        if Qt.Key_D in self._pressed_keys:
            target_wz -= 0.8
            
        # Ramping Logic (Acceleration Limiting)
        # Max Accel: 0.5 m/s^2. dt = 0.05s. step = 0.025
        lin_step = 0.025
        ang_step = 0.05
        
        if self._current_vx < target_vx:
            self._current_vx = min(target_vx, self._current_vx + lin_step)
        elif self._current_vx > target_vx:
            self._current_vx = max(target_vx, self._current_vx - lin_step)
            
        if self._current_wz < target_wz:
            self._current_wz = min(target_wz, self._current_wz + ang_step)
        elif self._current_wz > target_wz:
            self._current_wz = max(target_wz, self._current_wz - ang_step)
            
        # Deadband / Stop verification
        if abs(self._current_vx) < 0.01: self._current_vx = 0.0
        if abs(self._current_wz) < 0.01: self._current_wz = 0.0
            
        # Only send if there is motion or we are ramping down to stop
        if self._current_vx != 0.0 or self._current_wz != 0.0 or (target_vx == 0 and target_wz == 0):
             self._on_teleop_command(self._current_vx, self._current_wz)
    
    # ===== Callbacks =====
    
    def set_goal_callback(self, callback) -> None:
        """Set callback for goal requests."""
        self._goal_callback = callback
    
    def set_initial_pose_callback(self, callback) -> None:
        """Set callback for initial pose requests."""
        self._initial_pose_callback = callback
    
    def set_cancel_callback(self, callback) -> None:
        """Set callback for cancel requests."""
        self._cancel_callback = callback

    def set_teleop_callback(self, callback) -> None:
        """Set callback for teleop commands."""
        self._teleop_callback = callback
    
    def _on_goal_requested(self, x: float, y: float, theta: float) -> None:
        goal = NavigationGoal(pose=Pose2D(x=x, y=y, theta=theta))
        self.map_widget.set_goal(goal.pose)
        self.control_panel.set_goal_pose(goal.pose)
        self.status_bar.showMessage(f"Goal set: ({x:.2f}, {y:.2f})")
        
        if self._goal_callback:
            self._goal_callback(goal)
    
    def _on_initial_pose_requested(self, x: float, y: float, theta: float) -> None:
        pose = Pose2D(x=x, y=y, theta=theta)
        self.status_bar.showMessage(f"Initial pose set: ({x:.2f}, {y:.2f})")
        
        if self._initial_pose_callback:
            self._initial_pose_callback(pose)
    
    def _on_cancel_navigation(self) -> None:
        self.map_widget.clear_goal()
        self.status_bar.showMessage("Navigation cancelled")
        
        if self._cancel_callback:
            self._cancel_callback()

    def _on_teleop_command(self, linear: float, angular: float) -> None:
        """Handle teleop command."""
        if self._teleop_callback:
            self._teleop_callback(linear, angular)
    
    # ===== Data Update Methods =====
    
    def update_map(self, map_data: MapData) -> None:
        self.map_widget.set_map(map_data)
        self.control_panel.set_topic_connected("/map", True)
    
    def update_laser(self, laser_data: LaserScanData) -> None:
        self.map_widget.set_laser(laser_data)
        self.control_panel.set_topic_connected("/scan", True)
    
    def update_robot_pose(self, pose: Pose2D) -> None:
        self.map_widget.set_robot_pose(pose)
        self.control_panel.set_robot_pose(pose)
        self.control_panel.set_topic_connected("/odom", True)
    
    def update_global_path(self, path: PathData) -> None:
        self.map_widget.set_global_path(path)
        self.control_panel.set_topic_connected("/plan", True)
    
    def update_local_path(self, path: PathData) -> None:
        self.map_widget.set_local_path(path)
    
    def update_global_costmap(self, costmap: CostmapData) -> None:
        self.map_widget.set_global_costmap(costmap)
    
    def update_local_costmap(self, costmap: CostmapData) -> None:
        self.map_widget.set_local_costmap(costmap)
        self.control_panel.set_topic_connected("/local_costmap", True)
    
    def update_navigation_status(self, status: NavigationStatus) -> None:
        self.control_panel.set_navigation_status(status)

    # === Map I/O Callbacks ===

    def _on_save_map(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Map", "", "Map Files (*.yaml)"
        )
        if path:
            self.save_map_requested.emit(path)

    def _on_load_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Map", "", "Map Files (*.yaml)"
        )
        if path:
            self.load_map_requested.emit(path)
