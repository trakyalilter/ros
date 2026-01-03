
import json
import os
from typing import List, Tuple
from dataclasses import asdict
from ..domain.entities import RestrictedZone

from PyQt5.QtCore import QObject, pyqtSignal

class ZoneManager(QObject):
    """Manages Restricted Zones and validation."""
    
    zones_changed = pyqtSignal()
    
    def __init__(self, storage_path: str = None):
        super().__init__()
        if storage_path is None:
            home = os.path.expanduser("~")
            self.storage_dir = os.path.join(home, ".nav_viewer")
            self.storage_path = os.path.join(self.storage_dir, "zones.json")
        else:
            self.storage_path = storage_path
            self.storage_dir = os.path.dirname(storage_path)
            
        self._zones: List[RestrictedZone] = []
        self._ensure_storage_dir()
        self.load()
        
    def _ensure_storage_dir(self):
        if not os.path.exists(self.storage_dir):
            os.makedirs(self.storage_dir)
            
    def load(self):
        """Load zones from JSON storage."""
        if not os.path.exists(self.storage_path):
            return
            
        try:
            with open(self.storage_path, 'r') as f:
                data = json.load(f)
                self._zones = []
                for item in data:
                    # Convert list of lists to list of tuples
                    points = [tuple(p) for p in item['points']]
                    zone = RestrictedZone(
                        name=item['name'],
                        points=points,
                        id=item['id'],
                        is_active=item.get('is_active', True)
                    )
                    self._zones.append(zone)
            self.zones_changed.emit()
        except Exception as e:
            print(f"Error loading zones: {e}")
            
    def save(self):
        """Save zones to JSON storage."""
        data = []
        for zone in self._zones:
            z_dict = asdict(zone)
            data.append(z_dict)
            
        try:
            with open(self.storage_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Error saving zones: {e}")
            
    def get_zones(self) -> List[RestrictedZone]:
        return self._zones
        
    def add_zone(self, zone: RestrictedZone):
        self._zones.append(zone)
        self.save()
        self.zones_changed.emit()
        
    def remove_zone(self, zone_id: str):
        self._zones = [z for z in self._zones if z.id != zone_id]
        self.save()
        self.zones_changed.emit()
        
    def is_point_restricted(self, x: float, y: float) -> bool:
        """Check if a point is inside any active restricted zone."""
        for zone in self._zones:
            if not zone.is_active:
                continue
            if self._is_point_in_polygon(x, y, zone.points):
                return True
        return False
        
    def _is_point_in_polygon(self, x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
        """Ray casting algorithm for point in polygon test."""
        n = len(poly)
        if n < 3:
            return False
            
        inside = False
        p1x, p1y = poly[0]
        for i in range(1, n + 1):
            p2x, p2y = poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
