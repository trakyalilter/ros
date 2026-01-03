
import threading
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap

class NavigationServiceAdapter(QObject):
    """Adapter for General Navigation Services (Recovery, etc)."""
    
    action_finished = pyqtSignal(bool, str)  # success, message
    
    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        
        # Clients
        self._clear_global_client = node.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap'
        )
        self._clear_local_client = node.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        
    def clear_costmaps(self) -> None:
        """Clear both global and local costmaps."""
        # Run in thread to not block UI
        threading.Thread(target=self._clear_costmaps_thread, daemon=True).start()
        
    def _clear_costmaps_thread(self) -> None:
        try:
            # Check availability
            if not self._clear_global_client.wait_for_service(timeout_sec=1.0):
                self.action_finished.emit(False, "Global Costmap service not available")
                return
            if not self._clear_local_client.wait_for_service(timeout_sec=1.0):
                self.action_finished.emit(False, "Local Costmap service not available")
                return
                
            # Requests
            req_g = ClearEntireCostmap.Request()
            req_l = ClearEntireCostmap.Request()
            
            # Call synchronously in this thread
            future_g = self._clear_global_client.call_async(req_g)
            future_l = self._clear_local_client.call_async(req_l)
            
            # Note: We can't spin here because main thread is spinning. 
            # We must use call_async and wait? No, wait_for_future needs spin.
            # But the main thread handles spinning.
            # So we should just issue the calls and handle callbacks.
            # BUT, we want to know when BOTH are done.
            # Simpler: fire and forget, or chain them.
            
            # Let's chain them with callbacks on the MAIN thread to avoid threading complexity with spinning.
            # So, re-implement clear_costmaps to just make async calls.
            pass
            
        except Exception as e:
            self.action_finished.emit(False, f"Error: {e}")

    # Re-implementing non-threaded async approach
    def clear_costmaps_async(self) -> None:
        """Trigger async clearing."""
        if not self._clear_global_client.service_is_ready():
             self.action_finished.emit(False, "Global Costmap service not ready")
             return
             
        future = self._clear_global_client.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(self._on_global_cleared)
        
    def _on_global_cleared(self, future):
        try:
            future.result() # check for exception
            # Now clear local
            if not self._clear_local_client.service_is_ready():
                 self.action_finished.emit(False, "Local Costmap service not ready (Global cleared)")
                 return
            
            future_l = self._clear_local_client.call_async(ClearEntireCostmap.Request())
            future_l.add_done_callback(self._on_local_cleared)
            
        except Exception as e:
            self.action_finished.emit(False, f"Failed to clear global costmap: {e}")

    def _on_local_cleared(self, future):
        try:
            future.result()
            self.action_finished.emit(True, "Costmaps Cleared")
        except Exception as e:
            self.action_finished.emit(False, f"Failed to clear local costmap: {e}")
