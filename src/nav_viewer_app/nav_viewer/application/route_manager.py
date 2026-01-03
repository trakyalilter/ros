
import json
import os
from typing import List, Optional
from dataclasses import asdict
from ..domain.entities import Route

class RouteManager:
    """Manages persistence and CRUD for Routes."""
    
    def __init__(self, storage_path: str = None):
        if storage_path is None:
            home = os.path.expanduser("~")
            self.storage_dir = os.path.join(home, ".nav_viewer")
            self.storage_path = os.path.join(self.storage_dir, "routes.json")
        else:
            self.storage_path = storage_path
            self.storage_dir = os.path.dirname(storage_path)
            
        self._routes: List[Route] = []
        self._ensure_storage_dir()
        self.load()
        
    def _ensure_storage_dir(self):
        if not os.path.exists(self.storage_dir):
            os.makedirs(self.storage_dir)
            
    def load(self):
        """Load routes from JSON storage."""
        if not os.path.exists(self.storage_path):
            return
            
        try:
            with open(self.storage_path, 'r') as f:
                data = json.load(f)
                self._routes = []
                for item in data:
                    route = Route(
                        name=item['name'],
                        waypoints=item.get('waypoints', []),
                        loop_type=item.get('loop_type', 'once'),
                        loop_count=item.get('loop_count', 1),
                        id=item['id']
                    )
                    self._routes.append(route)
        except Exception as e:
            print(f"Error loading routes: {e}")
            
    def save(self):
        """Save routes to JSON storage."""
        data = []
        for route in self._routes:
            rt_dict = asdict(route)
            # Remove property if present (dataclasses don't include properties by default, but good to be safe)
            data.append(rt_dict)
            
        try:
            with open(self.storage_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Error saving routes: {e}")
            
    def get_routes(self) -> List[Route]:
        return self._routes
        
    def add_route(self, route: Route):
        self._routes.append(route)
        self.save()
        
    def update_route(self, route: Route):
        """Update existing route by ID."""
        for i, r in enumerate(self._routes):
            if r.id == route.id:
                self._routes[i] = route
                self.save()
                return
        
    def remove_route(self, route_id: str):
        self._routes = [r for r in self._routes if r.id != route_id]
        self.save()
        
    def get_route_by_id(self, route_id: str) -> Optional[Route]:
        for r in self._routes:
            if r.id == route_id:
                return r
        return None
