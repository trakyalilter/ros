import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/Desktop/ros2_ws/src/nav_viewer_app/install/nav_viewer'
