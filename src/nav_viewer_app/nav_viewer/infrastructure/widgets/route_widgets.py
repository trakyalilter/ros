
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QListWidget, QListWidgetItem, QPushButton, 
    QDialog, QLineEdit, QComboBox, QSpinBox, 
    QGroupBox, QAbstractItemView, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from ...domain.entities import Route, Station, LoopType
from ...application.route_manager import RouteManager
from ...application.station_manager import StationManager
from ..theme import Colors

class RouteEditorDialog(QDialog):
    """Dialog to create or edit a route."""
    
    def __init__(self, route: Route, station_manager: StationManager, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Route")
        self.setMinimumWidth(500)
        self.setMinimumHeight(400)
        
        self.route = route
        self.station_manager = station_manager
        
        self._setup_ui()
        self._load_data()
        
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit()
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # Stations Selection
        stations_group = QGroupBox("Waypoints")
        s_layout = QHBoxLayout(stations_group)
        
        # Available stations
        available_layout = QVBoxLayout()
        available_layout.addWidget(QLabel("Available Stations:"))
        self.available_list = QListWidget()
        self.available_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        available_layout.addWidget(self.available_list)
        s_layout.addLayout(available_layout)
        
        # Buttons
        btns_layout = QVBoxLayout()
        btns_layout.addStretch()
        self.btn_add = QPushButton(">>")
        self.btn_remove = QPushButton("<<")
        self.btn_up = QPushButton("Up")
        self.btn_down = QPushButton("Down")
        
        btns_layout.addWidget(self.btn_add)
        btns_layout.addWidget(self.btn_remove)
        btns_layout.addSpacing(20)
        btns_layout.addWidget(self.btn_up)
        btns_layout.addWidget(self.btn_down)
        btns_layout.addStretch()
        s_layout.addLayout(btns_layout)
        
        # Selected waypoints
        selected_layout = QVBoxLayout()
        selected_layout.addWidget(QLabel("Route Sequence:"))
        self.waypoints_list = QListWidget()
        selected_layout.addWidget(self.waypoints_list)
        s_layout.addLayout(selected_layout)
        
        layout.addWidget(stations_group)
        
        # Options
        opts_group = QGroupBox("Options")
        opts_layout = QHBoxLayout(opts_group)
        
        opts_layout.addWidget(QLabel("Loop:"))
        self.loop_combo = QComboBox()
        self.loop_combo.addItems(["Once", "Infinite", "Count"])
        opts_layout.addWidget(self.loop_combo)
        
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 100)
        self.count_spin.setEnabled(False)
        opts_layout.addWidget(QLabel("Times:"))
        opts_layout.addWidget(self.count_spin)
        opts_layout.addStretch()
        
        layout.addWidget(opts_group)
        
        # Dialog buttons
        dlg_btns = QHBoxLayout()
        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        
        dlg_btns.addStretch()
        dlg_btns.addWidget(cancel_btn)
        dlg_btns.addWidget(save_btn)
        layout.addLayout(dlg_btns)
        
        # Connections
        self.btn_add.clicked.connect(self._add_selected)
        self.btn_remove.clicked.connect(self._remove_selected)
        self.btn_up.clicked.connect(self._move_up)
        self.btn_down.clicked.connect(self._move_down)
        self.loop_combo.currentIndexChanged.connect(self._on_loop_changed)
        
    def _load_data(self):
        self.name_edit.setText(self.route.name)
        
        # Load available
        all_stations = self.station_manager.get_stations()
        for s in all_stations:
            item = QListWidgetItem(s.name)
            item.setData(Qt.UserRole, s.id)
            self.available_list.addItem(item)
            
        # Load waypoints
        for sid in self.route.waypoints:
            station = self.station_manager.get_station_by_id(sid)
            name = station.name if station else f"Unknown ({sid})"
            item = QListWidgetItem(name)
            item.setData(Qt.UserRole, sid)
            self.waypoints_list.addItem(item)
            
        # Options
        idx = 0
        if self.route.loop_type == "infinite": idx = 1
        elif self.route.loop_type == "count": idx = 2
        self.loop_combo.setCurrentIndex(idx)
        self.count_spin.setValue(self.route.loop_count)
        
    def _on_loop_changed(self, idx):
        self.count_spin.setEnabled(idx == 2)
        
    def _add_selected(self):
        for item in self.available_list.selectedItems():
            sid = item.data(Qt.UserRole)
            name = item.text()
            new_item = QListWidgetItem(name)
            new_item.setData(Qt.UserRole, sid)
            self.waypoints_list.addItem(new_item)
            
    def _remove_selected(self):
        for item in self.waypoints_list.selectedItems():
            self.waypoints_list.takeItem(self.waypoints_list.row(item))
            
    def _move_up(self):
        curr = self.waypoints_list.currentRow()
        if curr > 0:
            item = self.waypoints_list.takeItem(curr)
            self.waypoints_list.insertItem(curr - 1, item)
            self.waypoints_list.setCurrentRow(curr - 1)
            
    def _move_down(self):
        curr = self.waypoints_list.currentRow()
        if curr < self.waypoints_list.count() - 1:
            item = self.waypoints_list.takeItem(curr)
            self.waypoints_list.insertItem(curr + 1, item)
            self.waypoints_list.setCurrentRow(curr + 1)
            
    def get_route(self) -> Route:
        """Return updated route object."""
        self.route.name = self.name_edit.text()
        
        waypoints = []
        for i in range(self.waypoints_list.count()):
            item = self.waypoints_list.item(i)
            waypoints.append(item.data(Qt.UserRole))
        self.route.waypoints = waypoints
        
        idx = self.loop_combo.currentIndex()
        if idx == 0: self.route.loop_type = "once"
        elif idx == 1: self.route.loop_type = "infinite"
        elif idx == 2: self.route.loop_type = "count"
        
        self.route.loop_count = self.count_spin.value()
        return self.route


class RouteListWidget(QWidget):
    """Widget to list and manage routes."""
    
    run_route_requested = pyqtSignal(str) # route_id
    stop_route_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._routes = []
        self._route_manager: RouteManager = None
        self._station_manager: StationManager = None
        
        self._setup_ui()
        
    def set_managers(self, rm: RouteManager, sm: StationManager):
        self._route_manager = rm
        self._station_manager = sm
        self.refresh_list()
        
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Available Routes List
        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(f"""
            QListWidget {{
                background-color: {Colors.BG_DARK};
                border: 1px solid {Colors.BORDER};
                border-radius: 4px;
            }}
            QListWidget::item {{
                padding: 8px;
            }}
            QListWidget::item:selected {{
                background-color: {Colors.ACCENT_CYAN_DARK};
            }}
        """)
        layout.addWidget(self.list_widget)
        
        # CRUD Buttons
        btn_layout = QHBoxLayout()
        btn_new = QPushButton("New")
        btn_edit = QPushButton("Edit")
        btn_del = QPushButton("Delete")
        
        btn_new.clicked.connect(self._on_new)
        btn_edit.clicked.connect(self._on_edit)
        btn_del.clicked.connect(self._on_delete)
        
        btn_layout.addWidget(btn_new)
        btn_layout.addWidget(btn_edit)
        btn_layout.addWidget(btn_del)
        layout.addLayout(btn_layout)
        
        # Run Button
        self.btn_run = QPushButton("Run Route")
        self.btn_run.setObjectName("primaryButton")
        self.btn_run.clicked.connect(self._on_run)
        self.btn_run.setStyleSheet(f"""
            QPushButton {{
                background-color: {Colors.STATUS_SUCCESS};
                color: {Colors.TEXT_PRIMARY};
                font-weight: bold;
                padding: 10px;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background-color: #2ECC71;
            }}
        """)
        layout.addWidget(self.btn_run)
        
        # Stop Button
        self.btn_stop = QPushButton("Stop Execution")
        self.btn_stop.setObjectName("warningButton")
        self.btn_stop.clicked.connect(self.stop_route_requested.emit)
        self.btn_stop.setVisible(False)
        layout.addWidget(self.btn_stop)
        
    def refresh_list(self):
        self.list_widget.clear()
        if not self._route_manager: return
        
        self._routes = self._route_manager.get_routes()
        for route in self._routes:
            loop_text = route.loop_type.capitalize()
            if route.loop_type == "count":
                loop_text += f" ({route.loop_count})"
            text = f"{route.name}\n[{len(route.waypoints)} stops, {loop_text}]"
            
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, route.id)
            self.list_widget.addItem(item)
            
    def _on_new(self):
        new_route = Route(name="New Route")
        dlg = RouteEditorDialog(new_route, self._station_manager, self)
        if dlg.exec_() == QDialog.Accepted:
            self._route_manager.add_route(dlg.get_route())
            self.refresh_list()
            
    def _on_edit(self):
        item = self.list_widget.currentItem()
        if not item: return
        
        rid = item.data(Qt.UserRole)
        route = self._route_manager.get_route_by_id(rid)
        if route:
            dlg = RouteEditorDialog(route, self._station_manager, self)
            if dlg.exec_() == QDialog.Accepted:
                self._route_manager.update_route(dlg.get_route())
                self.refresh_list()

    def _on_delete(self):
        item = self.list_widget.currentItem()
        if not item: return
        
        rid = item.data(Qt.UserRole)
        res = QMessageBox.question(self, "Confirm Delete", "Delete this route?", 
                                   QMessageBox.Yes | QMessageBox.No)
        if res == QMessageBox.Yes:
            self._route_manager.remove_route(rid)
            self.refresh_list()
            
    def _on_run(self):
        item = self.list_widget.currentItem()
        if not item: return
        rid = item.data(Qt.UserRole)
        self.run_route_requested.emit(rid)
        
    def set_running_state(self, running: bool):
        self.btn_run.setVisible(not running)
        self.btn_stop.setVisible(running)
        self.list_widget.setEnabled(not running)
