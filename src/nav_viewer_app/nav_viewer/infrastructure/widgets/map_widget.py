"""
Map Visualization Widget

Interactive map display with pan/zoom, robot pose, laser scan,
paths, costmaps, and click-to-navigate functionality.
"""

import math
from typing import Optional, Tuple, List
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QSlider, QCheckBox, QFrame
)
from PyQt5.QtCore import Qt, QPoint, QPointF, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QImage, 
    QPixmap, QTransform, QPainterPath, QPolygonF,
    QWheelEvent, QMouseEvent, QFont, QLinearGradient
)

from ..theme import Colors
from ...domain.entities import (
    MapData, LaserScanData, Pose2D, PathData, CostmapData
)


class MapCanvas(QWidget):
    """
    Canvas widget for rendering the navigation map and overlays.
    Supports pan/zoom and click interactions.
    """
    
    # Signals
    goal_clicked = pyqtSignal(float, float, float)  # x, y, theta
    initial_pose_clicked = pyqtSignal(float, float, float)  # x, y, theta
    velocity_command = pyqtSignal(float, float)  # linear, angular
    polygon_completed = pyqtSignal(list)  # List[QPointF]
    station_created_at_point = pyqtSignal(QPointF) # point
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Data
        self._map_data: Optional[MapData] = None
        self._laser_data: Optional[LaserScanData] = None
        self._robot_pose: Optional[Pose2D] = None
        self._global_path: Optional[PathData] = None
        self._local_path: Optional[PathData] = None
        self._global_costmap: Optional[CostmapData] = None
        self._local_costmap: Optional[CostmapData] = None
        self._goal_pose: Optional[Pose2D] = None
        
        # View state
        self._scale = 50.0  # pixels per meter
        self._offset = QPointF(0, 0)  # pan offset in pixels
        self._rotation = 0.0
        
        # Interaction state
        self._dragging = False
        self._last_mouse_pos = QPoint(0, 0)
        self._setting_goal = False
        self._goal_start_pos: Optional[QPointF] = None
        self._teleop_mode = False
        self._teleop_start_pos: Optional[QPoint] = None
        self._teleop_start_pos: Optional[QPoint] = None
        self._drawing_mode = False
        self._setting_initial_pose = False
        self._initial_pose_start: Optional[QPointF] = None
        self._rect_start: Optional[QPointF] = None
        self._rect_current: Optional[QPointF] = None
        self._stations: List['Station'] = []
        self._zones: List['RestrictedZone'] = []
        
        # Visibility toggles
        self.show_laser = True
        self.show_global_path = True
        self.show_local_path = True
        self.show_global_costmap = False
        self.show_local_costmap = True
        self.show_grid = True
        
        # Cached images
        self._map_image: Optional[QImage] = None
        self._map_needs_update = True
        
        # Setup
        self.setMinimumSize(400, 400)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        
        # Background color
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(Colors.BG_DARKEST))
        self.setPalette(palette)
    
    # ==================== Data Setters ====================
    
    def set_map(self, map_data: MapData) -> None:
        """Update the occupancy grid map."""
        self._map_data = map_data
        self._map_needs_update = True
        self._update_map_image()
        self.update()
    
    def set_laser(self, laser_data: LaserScanData) -> None:
        """Update the laser scan data."""
        self._laser_data = laser_data
        self.update()
    
    def set_robot_pose(self, pose: Pose2D) -> None:
        """Update the robot pose."""
        self._robot_pose = pose
        self.update()
    
    def set_global_path(self, path: PathData) -> None:
        """Update the global path."""
        self._global_path = path
        self.update()
    
    def set_local_path(self, path: PathData) -> None:
        """Update the local path."""
        self._local_path = path
        self.update()
    
    def set_global_costmap(self, costmap: CostmapData) -> None:
        """Update the global costmap."""
        self._global_costmap = costmap
        self.update()
    
    def set_local_costmap(self, costmap: CostmapData) -> None:
        """Update the local costmap."""
        self._local_costmap = costmap
        self.update()
    
    def set_goal(self, pose: Pose2D) -> None:
        """Set the current navigation goal for display."""
        self._goal_pose = pose
        self.update()
    
    def clear_goal(self) -> None:
        """Clear the displayed goal."""
        self._goal_pose = None
        self.update()
    
    # ==================== Coordinate Transform ====================
    
    def world_to_screen(self, world_x: float, world_y: float) -> QPointF:
        """Convert world coordinates to screen coordinates."""
        # Apply scale and offset
        # Y is flipped because screen Y goes down, world Y goes up
        screen_x = world_x * self._scale + self._offset.x() + self.width() / 2
        screen_y = -world_y * self._scale + self._offset.y() + self.height() / 2
        return QPointF(screen_x, screen_y)
    
    def screen_to_world(self, screen_x: float, screen_y: float) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates."""
        world_x = (screen_x - self.width() / 2 - self._offset.x()) / self._scale
        world_y = -(screen_y - self.height() / 2 - self._offset.y()) / self._scale
        return (world_x, world_y)
    
    # ==================== Map Image Generation ====================
    
    def _update_map_image(self) -> None:
        """Generate a QImage from the map data."""
        if self._map_data is None or self._map_data.data is None:
            self._map_image = None
            return
        
        data = self._map_data.data
        height, width = data.shape
        
        # Create RGBA image
        image = QImage(width, height, QImage.Format_RGBA8888)
        
        # Color mapping
        for y in range(height):
            for x in range(width):
                val = data[y, x]
                if val == -1:  # Unknown
                    color = QColor(Colors.MAP_UNKNOWN)
                elif val == 0:  # Free
                    color = QColor(Colors.MAP_FREE)
                else:  # Occupied (scaling 0-100)
                    intensity = min(255, int(val * 2.55))
                    color = QColor(intensity, intensity, intensity)
                image.setPixelColor(x, height - 1 - y, color)  # Flip Y
        
        self._map_image = image
        self._map_needs_update = False
    
    # ==================== Painting ====================
    
    def paintEvent(self, event) -> None:
        """Render the map and all overlays."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw background
        painter.fillRect(self.rect(), QColor(Colors.BG_DARKEST))
        
        # Draw grid
        if self.show_grid:
            self._draw_grid(painter)
        
        # Draw map
        if self._map_image is not None:
            self._draw_map(painter)
        
        # Draw costmaps
        if self.show_global_costmap and self._global_costmap is not None:
            self._draw_costmap(painter, self._global_costmap, alpha=80)
        
        if self.show_local_costmap and self._local_costmap is not None:
            self._draw_costmap(painter, self._local_costmap, alpha=120)
        
        # Draw paths
        if self.show_global_path and self._global_path is not None:
            self._draw_path(painter, self._global_path, QColor(Colors.PATH_GLOBAL), 3)
        
        if self.show_local_path and self._local_path is not None:
            self._draw_path(painter, self._local_path, QColor(Colors.PATH_LOCAL), 2)
        
        # Draw laser scan
        if self.show_laser and self._laser_data is not None and self._robot_pose is not None:
            self._draw_laser(painter)
        
        # Draw goal
        if self._goal_pose is not None:
            self._draw_pose_marker(
                painter, self._goal_pose, 
                QColor(Colors.GOAL_COLOR), 
                size=25, 
                label="Goal"
            )
        
        # Draw robot
        if self._robot_pose is not None:
            self._draw_robot(painter)
        
        # Draw goal arrow while setting
        if self._setting_goal and self._goal_start_pos is not None:
            self._draw_goal_arrow(painter)
        
        # Draw scale indicator
        self._draw_scale_indicator(painter)
    
    def _draw_grid(self, painter: QPainter) -> None:
        """Draw reference grid."""
        painter.setPen(QPen(QColor(Colors.SURFACE_DARK), 1))
        
        # Calculate grid spacing (1 meter)
        grid_size = self._scale  # 1 meter in pixels
        
        # Get visible world bounds
        top_left = self.screen_to_world(0, 0)
        bottom_right = self.screen_to_world(self.width(), self.height())
        
        # Draw vertical lines
        x = math.floor(top_left[0])
        while x < bottom_right[0]:
            screen_pos = self.world_to_screen(x, 0)
            painter.drawLine(int(screen_pos.x()), 0, int(screen_pos.x()), self.height())
            x += 1
        
        # Draw horizontal lines
        y = math.floor(bottom_right[1])
        while y < top_left[1]:
            screen_pos = self.world_to_screen(0, y)
            painter.drawLine(0, int(screen_pos.y()), self.width(), int(screen_pos.y()))
            y += 1
    
    def _draw_map(self, painter: QPainter) -> None:
        """Draw the occupancy grid map."""
        if self._map_image is None or self._map_data is None:
            return
        
        # Calculate the map's screen position and size
        origin = self._map_data.origin
        resolution = self._map_data.resolution
        
        # Map dimensions in world coordinates
        map_width_world = self._map_data.width * resolution
        map_height_world = self._map_data.height * resolution
        
        # Get screen position of origin
        screen_origin = self.world_to_screen(origin.x, origin.y + map_height_world)
        
        # Scale the image
        scaled_width = int(map_width_world * self._scale)
        scaled_height = int(map_height_world * self._scale)
        
        if scaled_width > 0 and scaled_height > 0:
            scaled_image = self._map_image.scaled(
                scaled_width, scaled_height,
                Qt.IgnoreAspectRatio,
                Qt.FastTransformation
            )
            painter.drawImage(screen_origin, scaled_image)
    
    def _draw_costmap(self, painter: QPainter, costmap: CostmapData, alpha: int = 100) -> None:
        """Draw a costmap overlay."""
        if costmap.data is None:
            return
        
        data = costmap.data
        height, width = data.shape
        resolution = costmap.resolution
        
        for y in range(height):
            for x in range(width):
                val = data[y, x]
                if val > 0:  # Only draw non-free cells
                    world_x = x * resolution + costmap.origin.x
                    world_y = y * resolution + costmap.origin.y
                    screen_pos = self.world_to_screen(world_x, world_y)
                    
                    # Color based on cost
                    if val >= 254:  # Lethal
                        color = QColor(255, 0, 0, alpha)
                    elif val >= 253:  # Inscribed
                        color = QColor(255, 128, 0, alpha)
                    else:  # Inflation
                        intensity = min(255, int(val) * 2)
                        color = QColor(intensity, 0, intensity, alpha // 2)
                    
                    size = max(1, int(resolution * self._scale))
                    painter.fillRect(
                        int(screen_pos.x()), int(screen_pos.y()),
                        size, size, color
                    )
    
    def _draw_path(self, painter: QPainter, path: PathData, color: QColor, width: int) -> None:
        """Draw a navigation path."""
        if path.is_empty():
            return
        
        pen = QPen(color, width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(pen)
        
        points = []
        for pose in path.poses:
            screen_pos = self.world_to_screen(pose.x, pose.y)
            points.append(screen_pos)
        
        if len(points) >= 2:
            for i in range(len(points) - 1):
                painter.drawLine(points[i], points[i + 1])
    
    def _draw_laser(self, painter: QPainter) -> None:
        """Draw laser scan points."""
        if self._laser_data is None or self._robot_pose is None:
            return
        
        points = self._laser_data.get_cartesian_points()
        robot = self._robot_pose
        
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(Colors.LASER_COLOR)))
        
        cos_theta = math.cos(robot.theta)
        sin_theta = math.sin(robot.theta)
        
        for local_x, local_y in points:
            # Transform to world frame
            world_x = robot.x + local_x * cos_theta - local_y * sin_theta
            world_y = robot.y + local_x * sin_theta + local_y * cos_theta
            
            screen_pos = self.world_to_screen(world_x, world_y)
            painter.drawEllipse(screen_pos, 2, 2)
    
    def _draw_robot(self, painter: QPainter) -> None:
        """Draw the robot as an arrow."""
        if self._robot_pose is None:
            return
        
        pose = self._robot_pose
        screen_pos = self.world_to_screen(pose.x, pose.y)
        
        # Robot body size in pixels
        size = 20
        
        # Create arrow polygon
        arrow = QPolygonF([
            QPointF(size, 0),       # Tip
            QPointF(-size/2, -size/2),  # Left back
            QPointF(-size/3, 0),    # Back center
            QPointF(-size/2, size/2),   # Right back
        ])
        
        # Transform
        transform = QTransform()
        transform.translate(screen_pos.x(), screen_pos.y())
        transform.rotate(-math.degrees(pose.theta))  # Negative because screen Y is flipped
        
        transformed_arrow = transform.map(arrow)
        
        # Draw glow
        glow_pen = QPen(QColor(Colors.ROBOT_COLOR), 3)
        glow_pen.setColor(QColor(Colors.ACCENT_CYAN + "40"))
        painter.setPen(glow_pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPolygon(transformed_arrow)
        
        # Draw robot
        gradient = QLinearGradient(
            screen_pos.x() - size, screen_pos.y(),
            screen_pos.x() + size, screen_pos.y()
        )
        gradient.setColorAt(0, QColor(Colors.ACCENT_CYAN_DARK))
        gradient.setColorAt(1, QColor(Colors.ACCENT_CYAN))
        
        painter.setPen(QPen(QColor(Colors.TEXT_PRIMARY), 2))
        painter.setBrush(QBrush(gradient))
        painter.drawPolygon(transformed_arrow)
    
    def _draw_pose_marker(
        self, 
        painter: QPainter, 
        pose: Pose2D, 
        color: QColor,
        size: int = 20,
        label: str = ""
    ) -> None:
        """Draw a pose marker with arrow."""
        screen_pos = self.world_to_screen(pose.x, pose.y)
        
        # Draw arrow
        arrow = QPolygonF([
            QPointF(size, 0),
            QPointF(-size/2, -size/2),
            QPointF(-size/3, 0),
            QPointF(-size/2, size/2),
        ])
        
        transform = QTransform()
        transform.translate(screen_pos.x(), screen_pos.y())
        transform.rotate(-math.degrees(pose.theta))
        
        transformed_arrow = transform.map(arrow)
        
        painter.setPen(QPen(color.darker(120), 2))
        painter.setBrush(QBrush(color))
        painter.drawPolygon(transformed_arrow)
        
        # Draw label
        if label:
            painter.setPen(color)
            font = QFont("Segoe UI", 10, QFont.Bold)
            painter.setFont(font)
            painter.drawText(
                int(screen_pos.x()) + size + 5,
                int(screen_pos.y()) + 5,
                label
            )
    
    def _draw_goal_arrow(self, painter: QPainter) -> None:
        """Draw arrow while setting goal direction."""
        if self._goal_start_pos is None:
            return
        
        current_pos = self.mapFromGlobal(self.cursor().pos())
        
        painter.setPen(QPen(QColor(Colors.GOAL_COLOR), 2, Qt.DashLine))
        painter.drawLine(self._goal_start_pos, QPointF(current_pos))
    
    def _draw_scale_indicator(self, painter: QPainter) -> None:
        """Draw scale indicator in corner."""
        # 1 meter bar
        bar_length = self._scale
        margin = 20
        
        x = margin
        y = self.height() - margin
        
        painter.setPen(QPen(QColor(Colors.TEXT_SECONDARY), 2))
        painter.drawLine(x, y, int(x + bar_length), y)
        painter.drawLine(x, y - 5, x, y + 5)
        painter.drawLine(int(x + bar_length), y - 5, int(x + bar_length), y + 5)
        
        font = QFont("Segoe UI", 9)
        painter.setFont(font)
        painter.drawText(int(x + bar_length / 2 - 10), y - 10, "1m")
    
    # ==================== Interaction ====================
    
    def wheelEvent(self, event: QWheelEvent) -> None:
        """Handle zoom with mouse wheel."""
        delta = event.angleDelta().y()
        
        # Get mouse position before zoom
        mouse_pos = event.pos()
        world_before = self.screen_to_world(mouse_pos.x(), mouse_pos.y())
        
        # Apply zoom
        zoom_factor = 1.1 if delta > 0 else 0.9
        self._scale = max(5, min(500, self._scale * zoom_factor))
        
        # Adjust offset to keep mouse position fixed
        world_after = self.screen_to_world(mouse_pos.x(), mouse_pos.y())
        self._offset.setX(self._offset.x() + (world_after[0] - world_before[0]) * self._scale)
        self._offset.setY(self._offset.y() - (world_after[1] - world_before[1]) * self._scale)
        
        self.update()
    
    def mousePressEvent(self, event: QMouseEvent) -> None:
        """Handle mouse press for pan, goal, drawing, and pose setting."""
        if event.button() == Qt.LeftButton:
            if self._drawing_mode:
                world_pos = self.screen_to_world(event.pos().x(), event.pos().y())
                self._rect_start = QPointF(world_pos[0], world_pos[1])
                self._rect_current = self._rect_start
                self.update()
            elif self._setting_initial_pose:
                 # Start setting initial pose
                 self._initial_pose_start = QPointF(event.pos())
            elif event.modifiers() & Qt.ShiftModifier:
                # Teleop mode
                self._teleop_mode = True
                self._teleop_start_pos = event.pos()
                self.setCursor(Qt.ClosedHandCursor)
            else:
                # Pan mode
                self._dragging = True
                self._last_mouse_pos = event.pos()
                self.setCursor(Qt.ClosedHandCursor)
        elif event.button() == Qt.RightButton:
            # Start setting goal
            self._setting_goal = True
            self._goal_start_pos = QPointF(event.pos())
    
    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        """Handle mouse move for panning or teleop."""
        if self._teleop_mode and self._teleop_start_pos:
            delta = event.pos() - self._teleop_start_pos
            # Map Y (up is negative) to Linear X (forward is positive)
            # Map X (right is positive) to Angular Z (right is negative)
            
            # Sensitivity
            linear = -delta.y() * 0.005  # 200px = 1.0 m/s
            angular = -delta.x() * 0.01  # 100px = 1.0 rad/s (Increased sensitivity)
            
            # Clamp to robot limits (from nav2_params.yaml)
            linear = max(-0.5, min(0.5, linear))  # Robot max linear is ~0.5
            angular = max(-1.0, min(1.0, angular)) # Robot max angular is 1.0
            
            self.velocity_command.emit(linear, angular)
            
        elif self._dragging:
            delta = event.pos() - self._last_mouse_pos
            self._offset += QPointF(delta.x(), delta.y())
            self._last_mouse_pos = event.pos()
            self.update()
        elif self._setting_goal:
            self.update()  # Redraw goal arrow
        elif self._setting_initial_pose and self._initial_pose_start:
            self.update() # Redraw pose arrow
        elif self._drawing_mode and self._rect_start:
            world_pos = self.screen_to_world(event.pos().x(), event.pos().y())
            self._rect_current = QPointF(world_pos[0], world_pos[1])
            self.update()
    
    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        """Handle mouse release."""
        if event.button() == Qt.LeftButton:
            # Drawing Mode Release
            if self._drawing_mode and self._rect_start:
                # Finish rectangle
                if self._rect_start and self._rect_current:
                    x1, y1 = self._rect_start.x(), self._rect_start.y()
                    x2, y2 = self._rect_current.x(), self._rect_current.y()
                    
                    # Check minimum size
                    if abs(x2 - x1) > 0.1 and abs(y2 - y1) > 0.1:
                        # Create 4 points (CCW)
                        points = [
                            QPointF(min(x1, x2), min(y1, y2)), # BL
                            QPointF(max(x1, x2), min(y1, y2)), # BR
                            QPointF(max(x1, x2), max(y1, y2)), # TR
                            QPointF(min(x1, x2), max(y1, y2))  # TL
                        ]
                        self.polygon_completed.emit(points)
                        self.set_drawing_mode(False) # Auto-exit drawing mode
                        
                    self._rect_start = None
                    self._rect_current = None
                return  # Skip pan logic

            # Standard Left Click Release (Pan/Teleop)
            if self._teleop_mode:
                self._teleop_mode = False
                self._teleop_start_pos = None
                self.velocity_command.emit(0.0, 0.0)
            
            self._dragging = False
            self.setCursor(Qt.ArrowCursor)

            # Initial Pose Release
            if self._setting_initial_pose and self._initial_pose_start:
                 end_pos = event.pos()
                 start_world = self.screen_to_world(
                     self._initial_pose_start.x(), 
                     self._initial_pose_start.y()
                 )
                 end_world = self.screen_to_world(end_pos.x(), end_pos.y())
                 
                 dx = end_world[0] - start_world[0]
                 dy = end_world[1] - start_world[1]
                 
                 theta = 0.0
                 if abs(dx) > 0.1 or abs(dy) > 0.1:
                     theta = math.atan2(dy, dx)
                 
                 self.initial_pose_clicked.emit(start_world[0], start_world[1], theta)
                 self.set_initial_pose_mode(False) # Auto-exit
                 self._initial_pose_start = None
                 self.update()
                 return
                 
        elif event.button() == Qt.RightButton and self._setting_goal:
            self._setting_goal = False
            
            if self._goal_start_pos is not None:
                # Calculate goal pose
                start_world = self.screen_to_world(
                    self._goal_start_pos.x(), 
                    self._goal_start_pos.y()
                )
                end_world = self.screen_to_world(event.pos().x(), event.pos().y())
                
                # Calculate angle from drag direction
                dx = end_world[0] - start_world[0]
                dy = end_world[1] - start_world[1]
                
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    theta = math.atan2(dy, dx)
                else:
                    theta = 0.0
                
                self.goal_clicked.emit(start_world[0], start_world[1], theta)
            
            self._goal_start_pos = None
            self.update()
    
    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        """Handle double click for initial pose."""
        if event.button() == Qt.LeftButton:
            world_pos = self.screen_to_world(event.pos().x(), event.pos().y())
            self.initial_pose_clicked.emit(world_pos[0], world_pos[1], 0.0)
    
    def keyPressEvent(self, event) -> None:
        """Handle keyboard shortcuts."""
        if event.key() == Qt.Key_Home:
            # Reset view
            self._scale = 50.0
            self._offset = QPointF(0, 0)
            self.update()
        elif event.key() == Qt.Key_Plus or event.key() == Qt.Key_Equal:
            self._scale = min(500, self._scale * 1.2)
            self.update()
        elif event.key() == Qt.Key_Minus:
            self._scale = max(5, self._scale / 1.2)
            self.update()
        else:
            super().keyPressEvent(event)
    
    def center_on_robot(self) -> None:
        """Center the view on the robot."""
        if self._robot_pose is not None:
            self._offset = QPointF(
                -self._robot_pose.x * self._scale,
                self._robot_pose.y * self._scale
            )
            self.update()

    def set_drawing_mode(self, enabled: bool) -> None:
        """Enable or disable polygon drawing mode."""
        self._drawing_mode = enabled
        self._rect_start = None
        self._rect_current = None
        
        # Disable other modes
        if enabled:
            self._setting_initial_pose = False
            
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.update()

    def set_initial_pose_mode(self, enabled: bool) -> None:
        """Enable or disable initial pose setting."""
        self._setting_initial_pose = enabled
        self._initial_pose_start = None
        
        # Disable other modes
        if enabled:
            self._drawing_mode = False
            
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.update()

    def set_stations(self, stations: List['Station']) -> None:
        """Update list of stations to display."""
        self._stations = stations
        self.update()
        
    def set_zones(self, zones: List['RestrictedZone']) -> None:
        """Update list of zones to display."""
        self._zones = zones
        self.update()

    def _draw_stations(self, painter: QPainter) -> None:
        """Draw stations."""
        for station in self._stations:
            color = QColor(station.color)
            self._draw_pose_marker(
                painter, station.pose, 
                color, size=15, 
                label=station.name
            )

    def _draw_zones(self, painter: QPainter) -> None:
        """Draw restricted zones."""
        for zone in self._zones:
            if not zone.points or not zone.is_active:
                continue
                
            # Convert world points to screen points
            screen_points = []
            for pt in zone.points:
                screen_points.append(self.world_to_screen(pt[0], pt[1]))
            
            if len(screen_points) < 3:
                continue
                
            poly = QPolygonF(screen_points)
            
            # Semi-transparent red fill
            color = QColor(255, 0, 0, 40)
            border = QColor(255, 0, 0, 150)
            
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(border, 2, Qt.DashLine))
            painter.drawPolygon(poly)
            
            # Draw label
            center = poly.boundingRect().center()
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(center, zone.name)

    def _draw_current_rect(self, painter: QPainter) -> None:
        """Draw rectangle being drawn."""
        if not self._rect_start or not self._rect_current:
            return
            
        # Convert to screen coords
        start_screen = self.world_to_screen(self._rect_start.x(), self._rect_start.y())
        end_screen = self.world_to_screen(self._rect_current.x(), self._rect_current.y())
        
        rect = QRectF(start_screen, end_screen)
        
        # Draw fill and border
        color = QColor(255, 0, 0, 40)
        border = QColor(255, 0, 0, 150)
        
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(border, 2, Qt.DashLine))
        painter.drawRect(rect)
        
        # Dimensions text
        width = abs(self._rect_current.x() - self._rect_start.x())
        height = abs(self._rect_current.y() - self._rect_start.y())
        text = f"{width:.2f}m x {height:.2f}m"
        
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(rect.center(), text)

    def _draw_initial_pose_arrow(self, painter: QPainter) -> None:
        """Draw arrow while setting initial pose."""
        if self._initial_pose_start is None:
            return
            
        current_pos = self.mapFromGlobal(self.cursor().pos())
        
        painter.setPen(QPen(QColor(Colors.ACCENT_CYAN), 2, Qt.DashLine))
        painter.drawLine(self._initial_pose_start, QPointF(current_pos))

    # ==================== Interaction Overrides ====================

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        """Handle double click for station creation."""
        if not self._drawing_mode and event.button() == Qt.LeftButton:
            world_pos = self.screen_to_world(event.pos().x(), event.pos().y())
            point = QPointF(world_pos[0], world_pos[1])
            self.station_created_at_point.emit(point)
            
        super().mouseDoubleClickEvent(event)



    # mouseDoubleClickEvent removed for Rect mode


    def paintEvent(self, event) -> None:
        """Render the map and all overlays."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw background
        painter.fillRect(self.rect(), QColor(Colors.BG_DARKEST))
        
        # Draw grid
        if self.show_grid:
            self._draw_grid(painter)
        
        # Draw map
        if self._map_image is not None:
            self._draw_map(painter)
            
        # Draw Zones (Bottom Layer)
        self._draw_zones(painter)
        
        # Draw costmaps
        if self.show_global_costmap and self._global_costmap is not None:
            self._draw_costmap(painter, self._global_costmap, alpha=80)
        
        if self.show_local_costmap and self._local_costmap is not None:
            self._draw_costmap(painter, self._local_costmap, alpha=120)
        
        # Draw paths
        if self.show_global_path and self._global_path is not None:
            self._draw_path(painter, self._global_path, QColor(Colors.PATH_GLOBAL), 3)
        
        if self.show_local_path and self._local_path is not None:
            self._draw_path(painter, self._local_path, QColor(Colors.PATH_LOCAL), 2)
        
        # Draw laser scan
        if self.show_laser and self._laser_data is not None and self._robot_pose is not None:
            self._draw_laser(painter)
            
        # Draw Stations
        self._draw_stations(painter)
        
        # Draw Initial Pose Arrow
        if self._setting_initial_pose and self._initial_pose_start:
            self._draw_initial_pose_arrow(painter)

        # Draw goal
        if self._goal_pose is not None:
            self._draw_pose_marker(
                painter, self._goal_pose, 
                QColor(Colors.GOAL_COLOR), 
                size=25, 
                label="Goal"
            )
        
        # Draw robot
        if self._robot_pose is not None:
            self._draw_robot(painter)
            
        # Draw current drawing
        if self._drawing_mode:
            self._draw_current_rect(painter)
        
        # Draw goal arrow while setting
        if self._setting_goal and self._goal_start_pos is not None:
            self._draw_goal_arrow(painter)
        
        # Draw scale indicator
        self._draw_scale_indicator(painter)

class MapWidget(QWidget):
    """
    Complete map widget with canvas and controls.
    """
    
    goal_requested = pyqtSignal(float, float, float)
    initial_pose_requested = pyqtSignal(float, float, float)
    velocity_command = pyqtSignal(float, float)  # linear, angular
    polygon_completed = pyqtSignal(list)
    station_created_at_point = pyqtSignal(QPointF)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._setup_ui()
        self._connect_signals()
    
    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Canvas
        self.canvas = MapCanvas()
        layout.addWidget(self.canvas, stretch=1)
        
        # Controls bar
        controls = QFrame()
        controls.setStyleSheet(f"""
            QFrame {{
                background-color: {Colors.SURFACE_DARK};
                border-top: 1px solid {Colors.SURFACE_LIGHT};
                padding: 8px;
            }}
        """)
        controls_layout = QHBoxLayout(controls)
        controls_layout.setContentsMargins(12, 8, 12, 8)
        controls_layout.setSpacing(16)
        
        # Visibility toggles
        self.cb_laser = QCheckBox("Laser")
        self.cb_laser.setChecked(True)
        self.cb_global_path = QCheckBox("Global Path")
        self.cb_global_path.setChecked(True)
        self.cb_local_path = QCheckBox("Local Path")
        self.cb_local_path.setChecked(True)
        self.cb_local_costmap = QCheckBox("Local Costmap")
        self.cb_local_costmap.setChecked(True)
        self.cb_grid = QCheckBox("Grid")
        self.cb_grid.setChecked(True)
        
        controls_layout.addWidget(self.cb_laser)
        controls_layout.addWidget(self.cb_global_path)
        controls_layout.addWidget(self.cb_local_path)
        controls_layout.addWidget(self.cb_local_costmap)
        controls_layout.addWidget(self.cb_grid)
        controls_layout.addStretch()
        
        # Center button
        # Set Pose Button
        from PyQt5.QtWidgets import QPushButton
        self.btn_set_pose = QPushButton("2D Pose Estimate")
        self.btn_set_pose.setCheckable(True)
        self.btn_set_pose.setObjectName("actionButton") # Need to ensure styling supports checked state
        self.btn_set_pose.setStyleSheet("""
            QPushButton:checked {
                background-color: #00E5FF;
                color: #0F1724;
            }
        """)
        controls_layout.addWidget(self.btn_set_pose)

        # Center button
        self.btn_center = QPushButton("Center on Robot")
        self.btn_center.setObjectName("primaryButton")
        controls_layout.addWidget(self.btn_center)
        
        layout.addWidget(controls)
    
    def _connect_signals(self) -> None:
        # Checkbox signals
        self.cb_laser.toggled.connect(lambda v: setattr(self.canvas, 'show_laser', v) or self.canvas.update())
        self.cb_global_path.toggled.connect(lambda v: setattr(self.canvas, 'show_global_path', v) or self.canvas.update())
        self.cb_local_path.toggled.connect(lambda v: setattr(self.canvas, 'show_local_path', v) or self.canvas.update())
        self.cb_local_costmap.toggled.connect(lambda v: setattr(self.canvas, 'show_local_costmap', v) or self.canvas.update())
        self.cb_grid.toggled.connect(lambda v: setattr(self.canvas, 'show_grid', v) or self.canvas.update())
        
        # Center button
        self.btn_center.clicked.connect(self.canvas.center_on_robot)
        
        # Set Pose Button
        self.btn_set_pose.toggled.connect(self.canvas.set_initial_pose_mode)
        
        # Goal/pose signals
        self.canvas.goal_clicked.connect(self.goal_requested.emit)
        self.canvas.initial_pose_clicked.connect(self.initial_pose_requested.emit)
        self.canvas.initial_pose_clicked.connect(lambda: self.btn_set_pose.setChecked(False)) # Reset button
        self.canvas.velocity_command.connect(self.velocity_command.emit)
        self.canvas.polygon_completed.connect(self.polygon_completed.emit)
        self.canvas.station_created_at_point.connect(self.station_created_at_point.emit)
    
    # Delegate methods to canvas
    def set_map(self, map_data: MapData) -> None:
        self.canvas.set_map(map_data)
    
    def set_laser(self, laser_data: LaserScanData) -> None:
        self.canvas.set_laser(laser_data)
    
    def set_robot_pose(self, pose: Pose2D) -> None:
        self.canvas.set_robot_pose(pose)
    
    def set_global_path(self, path: PathData) -> None:
        self.canvas.set_global_path(path)
    
    def set_local_path(self, path: PathData) -> None:
        self.canvas.set_local_path(path)
    
    def set_global_costmap(self, costmap: CostmapData) -> None:
        self.canvas.set_global_costmap(costmap)
    
    def set_local_costmap(self, costmap: CostmapData) -> None:
        self.canvas.set_local_costmap(costmap)
    
    def set_goal(self, pose: Pose2D) -> None:
        self.canvas.set_goal(pose)
    
    def clear_goal(self) -> None:
        self.canvas.clear_goal()
        
    def set_stations(self, stations: List['Station']) -> None:
        self.canvas.set_stations(stations)
        
    def set_zones(self, zones: List['RestrictedZone']) -> None:
        self.canvas.set_zones(zones)
        
    def set_drawing_mode(self, enabled: bool) -> None:
        self.canvas.set_drawing_mode(enabled)
