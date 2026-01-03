
import os
import subprocess
import threading
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class MapAdapter(QObject):
    """Adapter for Nav2 Map Server using CLI tools."""
    
    save_finished = pyqtSignal(bool, str)  # success, message
    load_finished = pyqtSignal(bool, str)
    
    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._load_client = node.create_client(LoadMap, '/map_server/load_map')
        
    def save_map(self, file_path: str) -> None:
        """Save map using CLI in a separate thread."""
        threading.Thread(target=self._run_save_cli, args=(file_path,), daemon=True).start()
        
    def _run_save_cli(self, file_path: str) -> None:
        try:
            # Strip extension if present, CLI adds .yaml and .pgm
            base_name = os.path.splitext(file_path)[0]
            
            # Command: ros2 run nav2_map_server map_saver_cli -f <path> --fmt pgm
            env = os.environ.copy()
            # Ensure proper sourcing if needed, though running from app should inherit it.
            
            cmd = [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", base_name,
                "--fmt", "pgm",
                "--mode", "trinary",
                "--free", "0.25",
                "--occ", "0.65"
            ]
            
            result = subprocess.run(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            
            if result.returncode == 0:
                self.save_finished.emit(True, f"Map saved to {base_name}")
            else:
                self.save_finished.emit(False, f"Map save failed: {result.stderr}")
                
        except Exception as e:
            self.save_finished.emit(False, f"Error running map saver: {e}")

    def load_map(self, file_path: str) -> None:
        """Call load_map service (Load still needs service on Map Server)."""
        # Load usually works because map_server (loader) IS running (from navigation.launch -> nav2_bringup or slam_toolbox).
        # SLAM Toolbox also provides dynamic_map, but if we want to load a static map we need map_server.
        # If running SLAM, loading a static map might conflict.
        # But we'll try the standard Nav2 service.
        
        if not self._load_client.wait_for_service(timeout_sec=1.0):
            self.load_finished.emit(False, "Map Server load service not available")
            return
            
        req = LoadMap.Request()
        req.map_url = file_path
        
        future = self._load_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_load_done(f)
        )

    def _on_load_done(self, future):
        try:
            resp = future.result()
            if resp.result == LoadMap.Response.RESULT_SUCCESS:
                self.load_finished.emit(True, "Map loaded successfully")
            else:
                self.load_finished.emit(False, f"Map load failed with code {resp.result}")
        except Exception as e:
            self.load_finished.emit(False, f"Error calling service: {e}")
