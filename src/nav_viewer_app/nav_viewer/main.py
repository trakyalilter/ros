"""
Navigation Viewer - Main Entry Point

PyQt5 application for visualizing ROS2 navigation data.

Usage: 
    python -m nav_viewer.main           # Network mode (cross-container)
    python -m nav_viewer.main --local   # Local mode (same container)
"""

import sys
import os
import signal
import argparse

# Parse args for mode detection (don't set ROS_LOCALHOST_ONLY here!)
# That env var must be set in the SHELL before Python starts.
# Setting it in Python after DDS is already configured doesn't work.
def _early_setup():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--local', action='store_true')
    parser.add_argument('--domain-id', type=int, default=None)
    args, _ = parser.parse_known_args()
    
    is_local = args.local or os.environ.get('NAV_VIEWER_MODE') == 'local'
    
    # NOTE: Do NOT set ROS_LOCALHOST_ONLY here - it must be set before
    # Python starts. Use run.sh to set it in the shell environment.
    
    if args.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    
    return is_local

_IS_LOCAL = _early_setup()

# Now safe to import ROS2 and Qt modules
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

from .infrastructure.theme import apply_theme
from .infrastructure.main_window import MainWindow
from .application.coordinator import ApplicationCoordinator


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="ROS2 Navigation Viewer with PyQt5"
    )
    parser.add_argument(
        '--local', 
        action='store_true',
        help='Run in local mode (same container as ROS2). '
             'Uses ROS_LOCALHOST_ONLY=1 for faster discovery.'
    )
    parser.add_argument(
        '--domain-id',
        type=int,
        default=None,
        help='ROS domain ID (overrides ROS_DOMAIN_ID env var)'
    )
    return parser.parse_args()


def main():
    """Application entry point."""
    import rclpy
    from rclpy.node import Node
    
    # Use mode determined at import time
    mode_str = "LOCAL" if _IS_LOCAL else "NETWORK"
    
    # CRITICAL: Initialize ROS2 and create Node BEFORE Qt
    # This order is required for proper DDS/Qt interaction
    rclpy.init()
    ros_node = Node('nav_viewer_app')
    ros_node.get_logger().info(f"Navigation Viewer starting [{mode_str}]")
    
    # Enable high DPI scaling
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    # Create Qt application
    app = QApplication(sys.argv)
    app.setApplicationName("Navigation Viewer")
    app.setOrganizationName("ROS2")
    
    # Apply modern theme
    apply_theme(app)
    
    # Create main window with mode indicator
    window = MainWindow()
    window.setWindowTitle(f"ROS2 Navigation Viewer [{mode_str}]")
    window.show()
    
    # Create and start coordinator with pre-created node
    coordinator = ApplicationCoordinator(window, is_local=_IS_LOCAL)
    
    if not coordinator.start(ros_node):
        print("Warning: Failed to connect to ROS2. Running in offline mode.")
        window.status_bar.showMessage(f"[{mode_str}] ROS2 not connected - running in offline mode")
    else:
        window.status_bar.showMessage(f"[{mode_str}] Connected - Right-click on map to set goal")
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nShutting down...")
        coordinator.stop()
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Allow Python to handle signals during Qt event loop
    timer = app.startTimer(100)
    
    # Run application
    exit_code = app.exec_()
    
    # Cleanup
    coordinator.stop()
    
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
