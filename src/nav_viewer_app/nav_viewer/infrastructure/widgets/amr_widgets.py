
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QListWidget, 
    QListWidgetItem, QInputDialog, QMessageBox, QLabel, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from ...domain.entities import Station, Pose2D, RestrictedZone

class StationListWidget(QWidget):
    """Widget to list and manage stations."""
    
    goto_station_requested = pyqtSignal(Pose2D)
    save_station_requested = pyqtSignal(str) # name
    delete_station_requested = pyqtSignal(str) # id
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        
        # Header
        self._list_widget = QListWidget()
        self._layout.addWidget(self._list_widget)
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_save = QPushButton("Save Current Pose")
        self.btn_goto = QPushButton("Go To")
        self.btn_delete = QPushButton("Delete")
        
        btn_layout.addWidget(self.btn_save)
        btn_layout.addWidget(self.btn_goto)
        btn_layout.addWidget(self.btn_delete)
        self._layout.addLayout(btn_layout)
        
        # Connections
        self.btn_save.clicked.connect(self._on_save)
        self.btn_goto.clicked.connect(self._on_goto)
        self.btn_delete.clicked.connect(self._on_delete)
        
    def update_list(self, stations: list[Station]):
        self._list_widget.clear()
        for s in stations:
            item = QListWidgetItem(f"{s.name}")
            item.setData(Qt.UserRole, s.id)
            item.setData(Qt.UserRole + 1, s)
            self._list_widget.addItem(item)
            
    def _on_save(self):
        name, ok = QInputDialog.getText(self, "Save Station", "Station Name:")
        if ok and name:
            self.save_station_requested.emit(name)
            
    def _on_goto(self):
        item = self._list_widget.currentItem()
        if item:
            station = item.data(Qt.UserRole + 1)
            self.goto_station_requested.emit(station.pose)
            
    def _on_delete(self):
        item = self._list_widget.currentItem()
        if item:
            sid = item.data(Qt.UserRole)
            self.delete_station_requested.emit(sid)


class ZoneListWidget(QWidget):
    """Widget to manage restricted zones."""
    
    delete_zone_requested = pyqtSignal(str) # id
    toggle_draw_mode = pyqtSignal(bool) # enabled
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        
        self.lbl_status = QLabel("Mode: View")
        self._layout.addWidget(self.lbl_status)
        
        self._list_widget = QListWidget()
        self._layout.addWidget(self._list_widget)
        
        btn_layout = QHBoxLayout()
        self.btn_draw = QPushButton("Draw New Zone")
        self.btn_draw.setCheckable(True)
        self.btn_delete = QPushButton("Delete")
        
        btn_layout.addWidget(self.btn_draw)
        btn_layout.addWidget(self.btn_delete)
        self._layout.addLayout(btn_layout)
        
        self.btn_draw.toggled.connect(self._on_draw_toggled)
        self.btn_delete.clicked.connect(self._on_delete)
        
    def update_list(self, zones: list[RestrictedZone]):
        self._list_widget.clear()
        for z in zones:
            active = "[Active]" if z.is_active else "[Inactive]"
            item = QListWidgetItem(f"{z.name} {active}")
            item.setData(Qt.UserRole, z.id)
            self._list_widget.addItem(item)
            
    def _on_draw_toggled(self, checked):
        state = "Drawing" if checked else "View"
        self.lbl_status.setText(f"Mode: {state}")
        self.toggle_draw_mode.emit(checked)
        
    def _on_delete(self):
        item = self._list_widget.currentItem()
        if item:
            zid = item.data(Qt.UserRole)
            self.delete_zone_requested.emit(zid)
