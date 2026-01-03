
import json
import os
from typing import List, Optional
from dataclasses import asdict
from ..domain.entities import Station, Pose2D

class StationManager:
    """Manages persistence and CRUD for Stations."""
    
    def __init__(self, storage_path: str = None):
        if storage_path is None:
            home = os.path.expanduser("~")
            self.storage_dir = os.path.join(home, ".nav_viewer")
            self.storage_path = os.path.join(self.storage_dir, "stations.json")
        else:
            self.storage_path = storage_path
            self.storage_dir = os.path.dirname(storage_path)
            
        self._stations: List[Station] = []
        self._ensure_storage_dir()
        self.load()
        
    def _ensure_storage_dir(self):
        if not os.path.exists(self.storage_dir):
            os.makedirs(self.storage_dir)
            
    def load(self):
        """Load stations from JSON storage."""
        if not os.path.exists(self.storage_path):
            return
            
        try:
            with open(self.storage_path, 'r') as f:
                data = json.load(f)
                self._stations = []
                for item in data:
                    pose_data = item['pose']
                    pose = Pose2D(x=pose_data['x'], y=pose_data['y'], theta=pose_data['theta'])
                    station = Station(
                        name=item['name'],
                        pose=pose,
                        id=item['id'],
                        color=item.get('color', '#00ff00')
                    )
                    self._stations.append(station)
        except Exception as e:
            print(f"Error loading stations: {e}")
            
    def save(self):
        """Save stations to JSON storage."""
        data = []
        for station in self._stations:
            # Convert dataclass to dict, handle nested Pose2D
            st_dict = asdict(station)
            data.append(st_dict)
            
        try:
            with open(self.storage_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Error saving stations: {e}")
            
    def get_stations(self) -> List[Station]:
        return self._stations
        
    def add_station(self, station: Station):
        self._stations.append(station)
        self.save()
        
    def remove_station(self, station_id: str):
        self._stations = [s for s in self._stations if s.id != station_id]
        self.save()
        
    def get_station_by_id(self, station_id: str) -> Optional[Station]:
        for s in self._stations:
            if s.id == station_id:
                return s
        return None
