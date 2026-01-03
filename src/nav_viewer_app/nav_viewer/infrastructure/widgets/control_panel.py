"""
Control Panel Widget

Side panel with navigation status, controls, and topic information.
"""

import math
from typing import Optional

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QGroupBox, QFrame, QGridLayout,
    QDoubleSpinBox, QProgressBar, QScrollArea
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

from ..theme import Colors
from ...domain.entities import Pose2D, NavigationStatus, RobotState


class StatusIndicator(QFrame):
    """Small colored indicator dot."""
    
    def __init__(self, parent=None, connected: bool = False):
        super().__init__(parent)
        self.setFixedSize(12, 12)
        self.set_connected(connected)
    
    def set_connected(self, connected: bool) -> None:
        color = Colors.STATUS_SUCCESS if connected else Colors.STATUS_ERROR
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {color};
                border-radius: 6px;
            }}
        """)


class TopicStatusWidget(QWidget):
    """Shows connection status for a single topic."""
    
    def __init__(self, topic_name: str, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(8)
        
        self.indicator = StatusIndicator(connected=False)
        self.label = QLabel(topic_name)
        self.label.setStyleSheet(f"color: {Colors.TEXT_SECONDARY};")
        
        layout.addWidget(self.indicator)
        layout.addWidget(self.label, stretch=1)
    
    def set_connected(self, connected: bool) -> None:
        self.indicator.set_connected(connected)
        color = Colors.TEXT_PRIMARY if connected else Colors.TEXT_MUTED
        self.label.setStyleSheet(f"color: {color};")


class PoseDisplayWidget(QWidget):
    """Displays a pose with x, y, theta values."""
    
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        
        title_label = QLabel(title)
        title_label.setStyleSheet(f"""
            color: {Colors.TEXT_SECONDARY};
            font-size: 11px;
        """)
        layout.addWidget(title_label)
        
        coords_layout = QHBoxLayout()
        coords_layout.setSpacing(12)
        
        self.x_label = QLabel("X: --")
        self.y_label = QLabel("Y: --")
        self.theta_label = QLabel("θ: --")
        
        for label in [self.x_label, self.y_label, self.theta_label]:
            label.setStyleSheet(f"""
                color: {Colors.TEXT_PRIMARY};
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 12px;
            """)
            coords_layout.addWidget(label)
        
        coords_layout.addStretch()
        layout.addLayout(coords_layout)
    
    def set_pose(self, pose: Optional[Pose2D]) -> None:
        if pose is None:
            self.x_label.setText("X: --")
            self.y_label.setText("Y: --")
            self.theta_label.setText("θ: --")
        else:
            self.x_label.setText(f"X: {pose.x:.2f}")
            self.y_label.setText(f"Y: {pose.y:.2f}")
            self.theta_label.setText(f"θ: {math.degrees(pose.theta):.1f}°")


class ControlPanel(QWidget):
    """
    Side panel with navigation controls and status information.
    """
    
    # Signals
    cancel_navigation_clicked = pyqtSignal()
    send_goal_clicked = pyqtSignal(float, float, float)  # x, y, theta
    set_initial_pose_clicked = pyqtSignal(float, float, float)
    send_velocity = pyqtSignal(float, float)  # linear, angular
    clear_costmaps_clicked = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setFixedWidth(320)
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        from PyQt5.QtWidgets import QTabWidget
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: none;
                background-color: {Colors.BG_MEDIUM};
            }}
            QTabBar::tab {{
                background-color: {Colors.BG_DARKEST};
                color: {Colors.TEXT_SECONDARY};
                padding: 8px 16px;
                border: 1px solid {Colors.BORDER};
                border-bottom: none;
                border_top_left_radius: 4px;
                border_top_right_radius: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: {Colors.BG_MEDIUM};
                color: {Colors.ACCENT_CYAN};
                font-weight: bold;
            }}
        """)
        
        # === Tab 1: Dashboard (Original Content) ===
        dashboard = QWidget()
        dashboard_layout = QVBoxLayout(dashboard)
        dashboard_layout.setContentsMargins(16, 16, 16, 16)
        dashboard_layout.setSpacing(16)
        
        # ... (Reuse original components) ...
        # Header
        header = QLabel("Navigation Viewer")
        header.setStyleSheet(f"""
            font-size: 20px;
            font-weight: 600;
            color: {Colors.ACCENT_CYAN};
        """)
        dashboard_layout.addWidget(header)
        
        # Connection Status
        status_group = QGroupBox("Connection Status")
        status_layout = QVBoxLayout(status_group)
        status_layout.setSpacing(4)
        
        self.topic_map = TopicStatusWidget("/map")
        self.topic_scan = TopicStatusWidget("/scan")
        self.topic_odom = TopicStatusWidget("/odom")
        self.topic_plan = TopicStatusWidget("/plan")
        self.topic_costmap = TopicStatusWidget("/local_costmap")
        
        status_layout.addWidget(self.topic_map)
        status_layout.addWidget(self.topic_scan)
        status_layout.addWidget(self.topic_odom)
        status_layout.addWidget(self.topic_plan)
        status_layout.addWidget(self.topic_costmap)
        
        dashboard_layout.addWidget(status_group)
        
        # Robot State
        robot_group = QGroupBox("Robot State")
        robot_layout = QVBoxLayout(robot_group)
        
        self.robot_pose_display = PoseDisplayWidget("Current Pose")
        robot_layout.addWidget(self.robot_pose_display)
        
        nav_status_layout = QHBoxLayout()
        nav_status_layout.addWidget(QLabel("Status:"))
        self.nav_status_label = QLabel("Idle")
        self.nav_status_label.setStyleSheet(f"""
            color: {Colors.ACCENT_CYAN};
            font-weight: 600;
        """)
        nav_status_layout.addWidget(self.nav_status_label)
        nav_status_layout.addStretch()
        robot_layout.addLayout(nav_status_layout)
        
        dashboard_layout.addWidget(robot_group)
        
        # Navigation Goal
        goal_group = QGroupBox("Navigation Goal")
        goal_layout = QVBoxLayout(goal_group)
        
        self.goal_pose_display = PoseDisplayWidget("Target Pose")
        goal_layout.addWidget(self.goal_pose_display)
        
        input_layout = QGridLayout()
        input_layout.setSpacing(8)
        
        input_layout.addWidget(QLabel("X:"), 0, 0)
        self.goal_x_spin = QDoubleSpinBox()
        self.goal_x_spin.setRange(-100, 100)
        self.goal_x_spin.setDecimals(2)
        self.goal_x_spin.setSingleStep(0.5)
        input_layout.addWidget(self.goal_x_spin, 0, 1)
        
        input_layout.addWidget(QLabel("Y:"), 0, 2)
        self.goal_y_spin = QDoubleSpinBox()
        self.goal_y_spin.setRange(-100, 100)
        self.goal_y_spin.setDecimals(2)
        self.goal_y_spin.setSingleStep(0.5)
        input_layout.addWidget(self.goal_y_spin, 0, 3)
        
        input_layout.addWidget(QLabel("θ (deg):"), 1, 0)
        self.goal_theta_spin = QDoubleSpinBox()
        self.goal_theta_spin.setRange(-180, 180)
        self.goal_theta_spin.setDecimals(1)
        self.goal_theta_spin.setSingleStep(15)
        input_layout.addWidget(self.goal_theta_spin, 1, 1)
        
        goal_layout.addLayout(input_layout)
        
        buttons_layout = QHBoxLayout()
        self.btn_send_goal = QPushButton("Send Goal")
        self.btn_send_goal.setObjectName("primaryButton")
        self.btn_send_goal.clicked.connect(self._on_send_goal)
        
        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setObjectName("dangerButton")
        self.btn_cancel.clicked.connect(self.cancel_navigation_clicked.emit)
        
        buttons_layout.addWidget(self.btn_send_goal)
        buttons_layout.addWidget(self.btn_cancel)
        goal_layout.addLayout(buttons_layout)
        
        dashboard_layout.addWidget(goal_group)
        
        # Initial Pose
        pose_group = QGroupBox("Set Initial Pose")
        pose_layout = QVBoxLayout(pose_group)
        pose_input_layout = QGridLayout()
        pose_input_layout.setSpacing(8)
        
        pose_input_layout.addWidget(QLabel("X:"), 0, 0)
        self.init_x_spin = QDoubleSpinBox()
        self.init_x_spin.setRange(-100, 100)
        self.init_x_spin.setDecimals(2)
        pose_input_layout.addWidget(self.init_x_spin, 0, 1)
        
        pose_input_layout.addWidget(QLabel("Y:"), 0, 2)
        self.init_y_spin = QDoubleSpinBox()
        self.init_y_spin.setRange(-100, 100)
        self.init_y_spin.setDecimals(2)
        pose_input_layout.addWidget(self.init_y_spin, 0, 3)
        
        pose_input_layout.addWidget(QLabel("θ (deg):"), 1, 0)
        self.init_theta_spin = QDoubleSpinBox()
        self.init_theta_spin.setRange(-180, 180)
        self.init_theta_spin.setDecimals(1)
        pose_input_layout.addWidget(self.init_theta_spin, 1, 1)
        
        pose_layout.addLayout(pose_input_layout)
        self.btn_set_pose = QPushButton("Set Initial Pose")
        pose_layout.addWidget(self.btn_set_pose)
        dashboard_layout.addWidget(pose_group)
        
        # Manual Control
        teleop_group = QGroupBox("Manual Control")
        teleop_layout = QVBoxLayout(teleop_group)
        btn_grid = QGridLayout()
        btn_grid.setSpacing(8)
        
        self.btn_fwd = QPushButton("▲")
        self.btn_left = QPushButton("◀")
        self.btn_stop = QPushButton("●")
        self.btn_right = QPushButton("▶")
        self.btn_back = QPushButton("▼")
        
        for btn in [self.btn_fwd, self.btn_back, self.btn_left, self.btn_right, self.btn_stop]:
            btn.setFixedSize(40, 40)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setAutoRepeat(True)
            btn.setAutoRepeatInterval(100)
            btn.setStyleSheet(f"""
                font-size: 16px;
                font-weight: bold;
                background-color: {Colors.BG_LIGHT};
                border: 1px solid {Colors.BORDER};
                border-radius: 4px;
            """)
            
        self.btn_stop.setStyleSheet(f"""
            font-size: 16px;
            color: {Colors.STATUS_ERROR};
            font-weight: bold;
            background-color: {Colors.BG_LIGHT};
            border: 1px solid {Colors.STATUS_ERROR};
            border-radius: 4px;
        """)
        
        self.btn_fwd.pressed.connect(lambda: self.send_velocity.emit(0.2, 0.0))
        self.btn_back.pressed.connect(lambda: self.send_velocity.emit(-0.2, 0.0))
        self.btn_left.pressed.connect(lambda: self.send_velocity.emit(0.0, 0.4))
        self.btn_right.pressed.connect(lambda: self.send_velocity.emit(0.0, -0.4))
        self.btn_stop.pressed.connect(lambda: self.send_velocity.emit(0.0, 0.0))
        self.btn_fwd.released.connect(lambda: self.send_velocity.emit(0.0, 0.0))
        self.btn_back.released.connect(lambda: self.send_velocity.emit(0.0, 0.0))
        self.btn_left.released.connect(lambda: self.send_velocity.emit(0.0, 0.0))
        self.btn_right.released.connect(lambda: self.send_velocity.emit(0.0, 0.0))
        
        btn_grid.addWidget(self.btn_fwd, 0, 1)
        btn_grid.addWidget(self.btn_left, 1, 0)
        btn_grid.addWidget(self.btn_stop, 1, 1)
        btn_grid.addWidget(self.btn_right, 1, 2)
        btn_grid.addWidget(self.btn_back, 2, 1)
        
        teleop_layout.addLayout(btn_grid)
        key_hint = QLabel("Keyboard: W A S D")
        key_hint.setAlignment(Qt.AlignCenter)
        key_hint.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px;")
        teleop_layout.addWidget(key_hint)
        dashboard_layout.addWidget(teleop_group)
        
        # Help
        help_group = QGroupBox("Controls")
        help_layout = QVBoxLayout(help_group)
        help_text = QLabel(
            "• <b>Right-click + drag</b>: Set navigation goal\n"
            "• <b>Double-click</b>: Set initial pose\n"
            "• <b>Scroll</b>: Zoom in/out\n"
            "• <b>Click + drag</b>: Pan view\n"
            "• <b>Home</b>: Reset view"
        )
        help_text.setTextFormat(Qt.RichText)
        help_text.setWordWrap(True)
        help_text.setStyleSheet(f"color: {Colors.TEXT_SECONDARY}; font-size: 11px; line-height: 1.5;")
        help_layout.addWidget(help_text)
        dashboard_layout.addWidget(help_group)
        
        # Recovery
        recovery_group = self._create_recovery_group()
        dashboard_layout.addWidget(recovery_group)

        dashboard_layout.addStretch()
        
        # Scroll area for dashboard
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"""
            QScrollArea {{
                border: none;
                background-color: {Colors.BG_MEDIUM};
            }}
        """)
        scroll.setWidget(dashboard)
        
        # === Tab 2: Stations ===
        from .amr_widgets import StationListWidget
        self.station_list = StationListWidget()
        
        # === Tab 3: Zones ===
        from .amr_widgets import ZoneListWidget
        self.zone_list = ZoneListWidget()
        
        # === Tab 4: Routes ===
        from .route_widgets import RouteListWidget
        self.route_list = RouteListWidget()
        
        # Add tabs
        self.tabs.addTab(scroll, "Dashboard")
        self.tabs.addTab(self.station_list, "Stations")
        self.tabs.addTab(self.zone_list, "Zones")
        self.tabs.addTab(self.route_list, "Routes")
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(self.tabs)
    
    def _create_recovery_group(self) -> QGroupBox:
        from PyQt5.QtWidgets import QPushButton
        group = QGroupBox("Recovery")
        layout = QVBoxLayout(group)
        
        btn_clear = QPushButton("Clear Costmaps (Unstuck)")
        btn_clear.setObjectName("warningButton")
        btn_clear.clicked.connect(self.clear_costmaps_clicked.emit)
        
        layout.addWidget(btn_clear)
        return group
    
    def _on_send_goal(self) -> None:
        x = self.goal_x_spin.value()
        y = self.goal_y_spin.value()
        theta = math.radians(self.goal_theta_spin.value())
        self.send_goal_clicked.emit(x, y, theta)
    
    def _on_set_initial_pose(self) -> None:
        x = self.init_x_spin.value()
        y = self.init_y_spin.value()
        theta = math.radians(self.init_theta_spin.value())
        self.set_initial_pose_clicked.emit(x, y, theta)
    
    # ===== Public Update Methods =====
    
    def set_robot_pose(self, pose: Pose2D) -> None:
        self.robot_pose_display.set_pose(pose)
    
    def set_goal_pose(self, pose: Pose2D) -> None:
        self.goal_pose_display.set_pose(pose)
        self.goal_x_spin.setValue(pose.x)
        self.goal_y_spin.setValue(pose.y)
        self.goal_theta_spin.setValue(math.degrees(pose.theta))
    
    def set_navigation_status(self, status: NavigationStatus) -> None:
        status_colors = {
            NavigationStatus.IDLE: Colors.TEXT_SECONDARY,
            NavigationStatus.NAVIGATING: Colors.ACCENT_CYAN,
            NavigationStatus.SUCCEEDED: Colors.STATUS_SUCCESS,
            NavigationStatus.FAILED: Colors.STATUS_ERROR,
            NavigationStatus.CANCELED: Colors.STATUS_WARNING,
        }
        
        color = status_colors.get(status, Colors.TEXT_SECONDARY)
        self.nav_status_label.setText(status.value.capitalize())
        self.nav_status_label.setStyleSheet(f"""
            color: {color};
            font-weight: 600;
        """)
    
    def set_topic_connected(self, topic: str, connected: bool) -> None:
        topic_widgets = {
            "/map": self.topic_map,
            "/scan": self.topic_scan,
            "/odom": self.topic_odom,
            "/plan": self.topic_plan,
            "/local_costmap": self.topic_costmap,
        }
        
        widget = topic_widgets.get(topic)
        if widget:
            widget.set_connected(connected)
