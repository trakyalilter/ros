"""
Modern Dark Theme for Navigation Viewer

Premium dark theme with glassmorphism effects, vibrant accents,
and smooth gradients for a state-of-the-art UI experience.
"""

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt


# Color Palette
class Colors:
    """Modern color palette with vibrant accents."""
    
    # Background colors
    BG_DARKEST = "#0a0a14"
    BG_DARK = "#12121e"
    BG_MEDIUM = "#1a1a2e"
    BG_LIGHT = "#16213e"
    BG_LIGHTER = "#1f3460"
    
    # Surface colors (for cards, panels)
    SURFACE_DARK = "#1e1e32"
    SURFACE_MEDIUM = "#252540"
    SURFACE_LIGHT = "#2d2d4a"
    BORDER = "#2d2d4a"  # Added missing BORDER definition
    
    # Text colors
    TEXT_PRIMARY = "#ffffff"
    TEXT_SECONDARY = "#a0a0b2"
    TEXT_MUTED = "#6b6b80"
    TEXT_DISABLED = "#404052"
    
    # Accent colors
    ACCENT_CYAN = "#00d9ff"
    ACCENT_CYAN_DARK = "#00a8cc"
    ACCENT_MAGENTA = "#ff006e"
    ACCENT_MAGENTA_DARK = "#cc0058"
    ACCENT_PURPLE = "#9d4edd"
    ACCENT_GREEN = "#00f5a0"
    ACCENT_YELLOW = "#ffd60a"
    ACCENT_ORANGE = "#ff7b00"
    
    # Status colors
    STATUS_SUCCESS = "#00f5a0"
    STATUS_WARNING = "#ffd60a"
    STATUS_ERROR = "#ff006e"
    STATUS_INFO = "#00d9ff"
    
    # Map visualization colors
    MAP_FREE = "#1a1a2e"
    MAP_OCCUPIED = "#ffffff"
    MAP_UNKNOWN = "#0d0d14"
    
    # Navigation colors
    ROBOT_COLOR = "#00d9ff"
    GOAL_COLOR = "#ff006e"
    PATH_GLOBAL = "#00d9ff"
    PATH_LOCAL = "#ff006e"
    LASER_COLOR = "#00f5a0"
    COSTMAP_OBSTACLE = "#ff4444"
    COSTMAP_INFLATION = "#ff880088"


def get_stylesheet() -> str:
    """Generate the complete application stylesheet."""
    return f"""
    /* ========== Global Styles ========== */
    QWidget {{
        background-color: {Colors.BG_DARK};
        color: {Colors.TEXT_PRIMARY};
        font-family: 'Segoe UI', 'Inter', 'Roboto', sans-serif;
        font-size: 13px;
    }}
    
    QMainWindow {{
        background-color: {Colors.BG_DARKEST};
    }}
    
    /* ========== Menu Bar ========== */
    QMenuBar {{
        background-color: {Colors.BG_DARKEST};
        color: {Colors.TEXT_PRIMARY};
        border-bottom: 1px solid {Colors.SURFACE_DARK};
        padding: 4px 8px;
    }}
    
    QMenuBar::item {{
        background-color: transparent;
        padding: 6px 12px;
        border-radius: 4px;
    }}
    
    QMenuBar::item:selected {{
        background-color: {Colors.SURFACE_MEDIUM};
    }}
    
    QMenu {{
        background-color: {Colors.SURFACE_DARK};
        border: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 8px;
        padding: 4px;
    }}
    
    QMenu::item {{
        padding: 8px 24px;
        border-radius: 4px;
    }}
    
    QMenu::item:selected {{
        background-color: {Colors.ACCENT_CYAN}40;
        color: {Colors.ACCENT_CYAN};
    }}
    
    /* ========== Status Bar ========== */
    QStatusBar {{
        background-color: {Colors.BG_DARKEST};
        color: {Colors.TEXT_SECONDARY};
        border-top: 1px solid {Colors.SURFACE_DARK};
        padding: 4px;
    }}
    
    /* ========== Buttons ========== */
    QPushButton {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
            stop:0 {Colors.SURFACE_MEDIUM}, stop:1 {Colors.SURFACE_DARK});
        border: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 8px;
        padding: 10px 20px;
        color: {Colors.TEXT_PRIMARY};
        font-weight: 500;
    }}
    
    QPushButton:hover {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
            stop:0 {Colors.SURFACE_LIGHT}, stop:1 {Colors.SURFACE_MEDIUM});
        border-color: {Colors.ACCENT_CYAN}80;
    }}
    
    QPushButton:pressed {{
        background-color: {Colors.SURFACE_DARK};
        border-color: {Colors.ACCENT_CYAN};
    }}
    
    QPushButton:disabled {{
        background-color: {Colors.BG_MEDIUM};
        color: {Colors.TEXT_DISABLED};
        border-color: {Colors.SURFACE_DARK};
    }}
    
    QPushButton#primaryButton {{
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 {Colors.ACCENT_CYAN_DARK}, stop:1 {Colors.ACCENT_CYAN});
        border: none;
        color: {Colors.BG_DARKEST};
        font-weight: 600;
    }}
    
    QPushButton#primaryButton:hover {{
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 {Colors.ACCENT_CYAN}, stop:1 {Colors.ACCENT_CYAN});
    }}
    
    QPushButton#dangerButton {{
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 {Colors.ACCENT_MAGENTA_DARK}, stop:1 {Colors.ACCENT_MAGENTA});
        border: none;
        color: {Colors.TEXT_PRIMARY};
        font-weight: 600;
    }}
    
    /* ========== Group Box ========== */
    QGroupBox {{
        background-color: {Colors.SURFACE_DARK}80;
        border: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 12px;
        margin-top: 16px;
        padding: 16px;
        font-weight: 600;
    }}
    
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 16px;
        padding: 0 8px;
        color: {Colors.TEXT_PRIMARY};
        background-color: {Colors.SURFACE_DARK};
        border-radius: 4px;
    }}
    
    /* ========== Labels ========== */
    QLabel {{
        color: {Colors.TEXT_PRIMARY};
        background-color: transparent;
    }}
    
    QLabel#sectionTitle {{
        font-size: 16px;
        font-weight: 600;
        color: {Colors.ACCENT_CYAN};
    }}
    
    QLabel#statusConnected {{
        color: {Colors.STATUS_SUCCESS};
    }}
    
    QLabel#statusDisconnected {{
        color: {Colors.STATUS_ERROR};
    }}
    
    /* ========== Line Edit ========== */
    QLineEdit {{
        background-color: {Colors.SURFACE_DARK};
        border: 2px solid {Colors.SURFACE_LIGHT};
        border-radius: 8px;
        padding: 8px 12px;
        color: {Colors.TEXT_PRIMARY};
        selection-background-color: {Colors.ACCENT_CYAN}60;
    }}
    
    QLineEdit:focus {{
        border-color: {Colors.ACCENT_CYAN};
    }}
    
    /* ========== Spin Box ========== */
    QSpinBox, QDoubleSpinBox {{
        background-color: {Colors.SURFACE_DARK};
        border: 2px solid {Colors.SURFACE_LIGHT};
        border-radius: 8px;
        padding: 6px 12px;
        color: {Colors.TEXT_PRIMARY};
    }}
    
    QSpinBox:focus, QDoubleSpinBox:focus {{
        border-color: {Colors.ACCENT_CYAN};
    }}
    
    QSpinBox::up-button, QDoubleSpinBox::up-button {{
        background-color: {Colors.SURFACE_MEDIUM};
        border-left: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 0 8px 0 0;
        width: 20px;
    }}
    
    QSpinBox::down-button, QDoubleSpinBox::down-button {{
        background-color: {Colors.SURFACE_MEDIUM};
        border-left: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 0 0 8px 0;
        width: 20px;
    }}
    
    /* ========== Check Box ========== */
    QCheckBox {{
        color: {Colors.TEXT_PRIMARY};
        spacing: 8px;
    }}
    
    QCheckBox::indicator {{
        width: 20px;
        height: 20px;
        border-radius: 4px;
        border: 2px solid {Colors.SURFACE_LIGHT};
        background-color: {Colors.SURFACE_DARK};
    }}
    
    QCheckBox::indicator:checked {{
        background-color: {Colors.ACCENT_CYAN};
        border-color: {Colors.ACCENT_CYAN};
    }}
    
    QCheckBox::indicator:hover {{
        border-color: {Colors.ACCENT_CYAN}80;
    }}
    
    /* ========== Slider ========== */
    QSlider::groove:horizontal {{
        height: 6px;
        background-color: {Colors.SURFACE_DARK};
        border-radius: 3px;
    }}
    
    QSlider::handle:horizontal {{
        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
            stop:0 {Colors.ACCENT_CYAN}, stop:1 {Colors.ACCENT_CYAN_DARK});
        width: 18px;
        height: 18px;
        margin: -6px 0;
        border-radius: 9px;
    }}
    
    QSlider::sub-page:horizontal {{
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 {Colors.ACCENT_CYAN_DARK}, stop:1 {Colors.ACCENT_CYAN});
        border-radius: 3px;
    }}
    
    /* ========== Scroll Bar ========== */
    QScrollBar:vertical {{
        background-color: {Colors.BG_DARK};
        width: 12px;
        border-radius: 6px;
    }}
    
    QScrollBar::handle:vertical {{
        background-color: {Colors.SURFACE_LIGHT};
        border-radius: 6px;
        min-height: 30px;
    }}
    
    QScrollBar::handle:vertical:hover {{
        background-color: {Colors.ACCENT_CYAN}60;
    }}
    
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
    }}
    
    QScrollBar:horizontal {{
        background-color: {Colors.BG_DARK};
        height: 12px;
        border-radius: 6px;
    }}
    
    QScrollBar::handle:horizontal {{
        background-color: {Colors.SURFACE_LIGHT};
        border-radius: 6px;
        min-width: 30px;
    }}
    
    QScrollBar::handle:horizontal:hover {{
        background-color: {Colors.ACCENT_CYAN}60;
    }}
    
    /* ========== Splitter ========== */
    QSplitter::handle {{
        background-color: {Colors.SURFACE_DARK};
    }}
    
    QSplitter::handle:horizontal {{
        width: 2px;
    }}
    
    QSplitter::handle:vertical {{
        height: 2px;
    }}
    
    /* ========== Tab Widget ========== */
    QTabWidget::pane {{
        background-color: {Colors.SURFACE_DARK};
        border: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 8px;
    }}
    
    QTabBar::tab {{
        background-color: {Colors.BG_MEDIUM};
        color: {Colors.TEXT_SECONDARY};
        padding: 10px 20px;
        border: 1px solid {Colors.SURFACE_DARK};
        border-bottom: none;
        border-top-left-radius: 8px;
        border-top-right-radius: 8px;
    }}
    
    QTabBar::tab:selected {{
        background-color: {Colors.SURFACE_DARK};
        color: {Colors.ACCENT_CYAN};
        border-color: {Colors.SURFACE_LIGHT};
    }}
    
    QTabBar::tab:hover:!selected {{
        background-color: {Colors.SURFACE_DARK}80;
    }}
    
    /* ========== Progress Bar ========== */
    QProgressBar {{
        background-color: {Colors.SURFACE_DARK};
        border: none;
        border-radius: 4px;
        height: 8px;
        text-align: center;
    }}
    
    QProgressBar::chunk {{
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 {Colors.ACCENT_CYAN_DARK}, stop:1 {Colors.ACCENT_CYAN});
        border-radius: 4px;
    }}
    
    /* ========== Frame ========== */
    QFrame#statusIndicator {{
        background-color: {Colors.STATUS_SUCCESS};
        border-radius: 5px;
        min-width: 10px;
        max-width: 10px;
        min-height: 10px;
        max-height: 10px;
    }}
    
    QFrame#statusIndicatorOff {{
        background-color: {Colors.STATUS_ERROR};
        border-radius: 5px;
        min-width: 10px;
        max-width: 10px;
        min-height: 10px;
        max-height: 10px;
    }}
    
    /* ========== Tool Tip ========== */
    QToolTip {{
        background-color: {Colors.SURFACE_DARK};
        color: {Colors.TEXT_PRIMARY};
        border: 1px solid {Colors.SURFACE_LIGHT};
        border-radius: 4px;
        padding: 6px 10px;
    }}
    """


def apply_theme(app: QApplication) -> None:
    """Apply the modern dark theme to the application."""
    app.setStyle("Fusion")
    app.setStyleSheet(get_stylesheet())
    
    # Set application font
    font = QFont("Segoe UI", 10)
    app.setFont(font)
