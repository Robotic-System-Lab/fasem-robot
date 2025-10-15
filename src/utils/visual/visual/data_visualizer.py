import sys
import os
import json
import glob
import datetime
import subprocess
import platform
import time
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QWidget,
    QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QFileDialog, QLabel,
    QFrame, QScrollArea, QComboBox, QListWidget, QListWidgetItem, QSplitter,
    QMessageBox, QProgressBar
)
from PySide6.QtGui import QWheelEvent, QColor, QBrush, QPainter, QPixmap, QShortcut, QKeySequence
from PySide6.QtCore import Qt, QRectF, QThread, Signal, QTimer
import random


# Custom GraphicsView dengan zoom dan drag
class GridView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
    
    def wheelEvent(self, event: QWheelEvent):
        if event.angleDelta().y() > 0:
            factor = 1.25
        else:
            factor = 0.8
        self.scale(factor, factor)


class ImageGeneratorThread(QThread):
    progress_updated = Signal(int)
    generation_completed = Signal(str)
    error_occurred = Signal(str)
    
    def __init__(self, data_list, output_folder):
        super().__init__()
        self.data_list = data_list
        self.output_folder = output_folder
    
    def run(self):
        try:
            os.makedirs(self.output_folder, exist_ok=True)
            paths = ['full', 'painted_edge', 'plain_edge']
            
            for path in paths:
                folder = os.path.join(self.output_folder, path)
                os.makedirs(folder, exist_ok=True)
            
            total_files = len(self.data_list)
            
            for idx, data in enumerate(self.data_list):
                self._generate_images_for_data(data)
                progress = int((idx + 1) * 100 / total_files)
                self.progress_updated.emit(progress)
            
            self.generation_completed.emit(self.output_folder)
            
        except Exception as e:
            self.error_occurred.emit(str(e))
    
    def _generate_images_for_data(self, data):
        """Generate three types of images from map data"""
        grid_width = data['info']['width']
        grid_height = data['info']['height']
        grid_data = data['data']
        timestamp = data['timestamp']
        
        if grid_width == 0 or grid_height == 0:
            return
        
        cell_size = 800 / max(grid_width, grid_height)
        
        # Create pixmaps for each type
        pixmap = QPixmap(800, 800)
        pixmap.fill(Qt.transparent)
        pixmaps = [pixmap.copy() for _ in range(3)]
        painters = [QPainter(p) for p in pixmaps]
        
        for row in range(grid_height):
            real_row = grid_height - 1 - row
            for col in range(grid_width):
                index = real_row * grid_width + col
                if index < len(grid_data):
                    value = grid_data[index]
                    
                    if value == 100:
                        color = [QColor(50, 50, 50) for _ in range(3)]
                    elif value == 101:
                        color = [QColor(50, 250, 50) for _ in range(3)]
                    elif value == 102:
                        color = [QColor(200, 200, 50) for _ in range(3)]
                    elif value == 103:
                        color = [QColor(250, 50, 50) for _ in range(3)]
                    elif value == -1:
                        color = [QColor('darkgray'), None, None]
                    elif value == 0:
                        color = [QColor('lightgray'), None, None]
                    else:
                        color = [None, None, None]
                    
                    for i, fill in enumerate(color):
                        if fill is not None:
                            rect = QRectF(col * cell_size, row * cell_size,
                                        cell_size, cell_size)
                            painters[i].fillRect(rect, QBrush(fill))
        
        for painter in painters:
            painter.end()
        
        # Save images
        paths = ['full', 'painted_edge', 'plain_edge']
        for i, path in enumerate(paths):
            folder = os.path.join(self.output_folder, path)
            full_path = os.path.join(folder, f"{timestamp}.png")
            pixmaps[i].save(full_path, "PNG")


class DataVisualizerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Data Visualizer - FASEM Robot (Ctrl+F: Search, Ctrl+O: Browse, Ctrl+E: Open Folder, Ctrl+L: Live Monitor, F5: Refresh, Space: Play/Pause)")
        self.setGeometry(100, 100, 1200, 900)
        
        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel for controls
        left_panel = self._create_left_panel()
        splitter.addWidget(left_panel)
        
        # Right panel for visualization
        right_panel = self._create_right_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter proportions
        splitter.setSizes([300, 900])
        
        # Initialize variables
        self.current_folder = ""
        self.data_files = []
        self.current_data = None
        self.all_folders = []  # Store all available folders for filtering
        self.auto_timer = QTimer()
        self.auto_timer.timeout.connect(self._auto_next_frame)
        
        # Live monitor timer
        self.live_monitor_timer = QTimer()
        self.live_monitor_timer.timeout.connect(self._live_monitor_update)
        self.live_monitor_enabled = False
        
        # Live monitor configuration
        self.file_age_threshold = 0.5  # Skip files newer than 0.5 seconds
        self.json_load_retries = 3
        self.json_retry_delay = 0.1
        
        # Load available folders
        self._load_available_folders()
        
        # Setup keyboard shortcuts
        self._setup_shortcuts()
    
    def _create_left_panel(self):
        """Create the left control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Folder selection
        layout.addWidget(QLabel("Select Session Folder:"))
        
        # Search box for filtering folders
        search_layout = QHBoxLayout()
        search_layout.addWidget(QLabel("Search:"))
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Filter folders by name... (Ctrl+F)")
        self.search_box.textChanged.connect(self._filter_folders)
        self.search_box.setToolTip("Type to filter session folders by name")
        search_layout.addWidget(self.search_box)
        layout.addLayout(search_layout)
        
        self.folder_combo = QComboBox()
        self.folder_combo.currentTextChanged.connect(self._folder_changed)
        layout.addWidget(self.folder_combo)
        
        # Folder management buttons
        folder_buttons_layout = QHBoxLayout()
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._load_available_folders)
        refresh_btn.setToolTip("Refresh the list of available session folders")
        
        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self._browse_folder)
        browse_btn.setToolTip("Browse and select a session folder (Ctrl+O)")
        
        open_folder_btn = QPushButton("Open Folder")
        open_folder_btn.clicked.connect(self._open_txt_map_folder)
        open_folder_btn.setToolTip("Open txt_map folder in file manager (Ctrl+E)")
        
        folder_buttons_layout.addWidget(refresh_btn)
        folder_buttons_layout.addWidget(browse_btn)
        layout.addLayout(folder_buttons_layout)
        layout.addWidget(open_folder_btn)
        
        # File list
        layout.addWidget(QLabel("Data Files:"))
        self.file_list = QListWidget()
        self.file_list.itemClicked.connect(self._file_selected)
        layout.addWidget(self.file_list)
        
        # Live monitor controls
        layout.addWidget(QLabel("Live Monitor:"))
        live_monitor_layout = QHBoxLayout()
        
        self.live_monitor_btn = QPushButton("Start Live Monitor")
        self.live_monitor_btn.clicked.connect(self._toggle_live_monitor)
        self.live_monitor_btn.setToolTip("Monitor folder for new data files and auto-update (Ctrl+L)")
        
        self.auto_latest_checkbox = QPushButton("Auto Show Latest")
        self.auto_latest_checkbox.setCheckable(True)
        self.auto_latest_checkbox.setChecked(True)
        self.auto_latest_checkbox.setToolTip("Automatically display the latest JSON file when live monitoring")
        
        # Live monitor status indicator
        self.live_status_label = QLabel("● Monitoring: Off")
        self.live_status_label.setStyleSheet("color: gray;")
        
        live_monitor_layout.addWidget(self.live_monitor_btn)
        live_monitor_layout.addWidget(self.auto_latest_checkbox)
        layout.addLayout(live_monitor_layout)
        layout.addWidget(self.live_status_label)
        
        # Playback controls
        layout.addWidget(QLabel("Playback Controls:"))
        
        controls_layout = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self._toggle_playback)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self._stop_playback)
        controls_layout.addWidget(self.play_btn)
        controls_layout.addWidget(self.stop_btn)
        layout.addLayout(controls_layout)
        
        # Speed control
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed (ms):"))
        self.speed_input = QLineEdit("500")
        speed_layout.addWidget(self.speed_input)
        layout.addLayout(speed_layout)
        
        # Generate evaluation images
        layout.addWidget(QLabel("Generate Images:"))
        self.generate_btn = QPushButton("Generate Evaluation Images")
        self.generate_btn.clicked.connect(self._generate_evaluation_images)
        layout.addWidget(self.generate_btn)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Status label
        self.status_label = QLabel("Ready")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        return panel
    
    def _create_right_panel(self):
        """Create the right visualization panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Grid display
        self.scene = QGraphicsScene()
        self.view = GridView(self.scene)
        self.view.setFixedSize(800, 800)
        layout.addWidget(self.view)
        
        # Info display
        self.info_label = QLabel("No data loaded")
        layout.addWidget(self.info_label)
        
        # Generate default grid
        self._generate_default_grid()
        
        return panel
    
    def _setup_shortcuts(self):
        """Setup keyboard shortcuts"""
        # Ctrl+F to focus search box
        search_shortcut = QShortcut(QKeySequence("Ctrl+F"), self)
        search_shortcut.activated.connect(lambda: self.search_box.setFocus())
        
        # Ctrl+O to browse folder
        browse_shortcut = QShortcut(QKeySequence("Ctrl+O"), self)
        browse_shortcut.activated.connect(self._browse_folder)
        
        # Ctrl+E to open txt_map folder
        open_folder_shortcut = QShortcut(QKeySequence("Ctrl+E"), self)
        open_folder_shortcut.activated.connect(self._open_txt_map_folder)
        
        # F5 to refresh
        refresh_shortcut = QShortcut(QKeySequence("F5"), self)
        refresh_shortcut.activated.connect(self._load_available_folders)
        
        # Space to toggle playback
        play_shortcut = QShortcut(QKeySequence("Space"), self)
        play_shortcut.activated.connect(self._toggle_playback)
        
        # Ctrl+L to toggle live monitor
        live_monitor_shortcut = QShortcut(QKeySequence("Ctrl+L"), self)
        live_monitor_shortcut.activated.connect(self._toggle_live_monitor)
        
        # Escape to clear search
        clear_search_shortcut = QShortcut(QKeySequence("Escape"), self)
        clear_search_shortcut.activated.connect(lambda: self.search_box.clear())
    
    def _generate_default_grid(self):
        """Generate a default grid display"""
        pixmap = QPixmap(800, 800)
        pixmap.fill(Qt.lightGray)
        painter = QPainter(pixmap)
        painter.setPen(QColor(Qt.darkGray))
        
        # Draw grid lines
        for i in range(0, 801, 20):
            painter.drawLine(i, 0, i, 800)
            painter.drawLine(0, i, 800, i)
        
        painter.end()
        self.scene.clear()
        self.scene.addPixmap(pixmap)
    
    def _load_available_folders(self):
        """Load available timestamp folders"""
        base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map")
        
        if not os.path.exists(base_path):
            self.status_label.setText("No txt_map folder found")
            self.all_folders = []
            return
        
        folders = []
        for item in os.listdir(base_path):
            folder_path = os.path.join(base_path, item)
            if os.path.isdir(folder_path):
                folders.append(item)
        
        folders.sort(reverse=True)  # Most recent first
        self.all_folders = folders
        
        # Apply current search filter
        self._filter_folders()
        
        if folders:
            self.status_label.setText(f"Found {len(folders)} session folders total")
        else:
            self.status_label.setText("No session folders found")
    
    def _filter_folders(self):
        """Filter folders based on search text"""
        search_text = self.search_box.text().lower()
        
        if search_text:
            filtered_folders = [folder for folder in self.all_folders 
                              if search_text in folder.lower()]
        else:
            filtered_folders = self.all_folders
        
        # Remember current selection
        current_selection = self.folder_combo.currentText()
        
        # Update combo box
        self.folder_combo.clear()
        self.folder_combo.addItems(filtered_folders)
        
        # Restore selection if still visible
        if current_selection in filtered_folders:
            index = self.folder_combo.findText(current_selection)
            if index >= 0:
                self.folder_combo.setCurrentIndex(index)
        
        # Update status
        if search_text:
            self.status_label.setText(f"Showing {len(filtered_folders)} of {len(self.all_folders)} folders")
        else:
            self.status_label.setText(f"Showing all {len(self.all_folders)} folders")
    
    def _browse_folder(self):
        """Browse and select a folder from txt_map directory"""
        base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map")
        
        # Create txt_map directory if it doesn't exist
        os.makedirs(base_path, exist_ok=True)
        
        # Open file dialog starting from txt_map directory
        folder_path = QFileDialog.getExistingDirectory(
            self,
            "Select Session Folder",
            base_path,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
        )
        
        if folder_path:
            # Extract folder name from full path
            folder_name = os.path.basename(folder_path)
            
            # Check if the selected folder is within txt_map
            if base_path in folder_path:
                # Refresh the folder list to include any new folders
                self._load_available_folders()
                
                # Clear search to show all folders
                self.search_box.clear()
                
                # Set as current selection
                index = self.folder_combo.findText(folder_name)
                if index >= 0:
                    self.folder_combo.setCurrentIndex(index)
                else:
                    QMessageBox.warning(self, "Folder Not Found", 
                                      f"Folder '{folder_name}' not found in the list.")
                
                self.status_label.setText(f"Selected folder: {folder_name}")
            else:
                QMessageBox.warning(self, "Invalid Selection", 
                                  "Please select a folder within the txt_map directory.")
    
    def _open_txt_map_folder(self):
        """Open the txt_map folder in system file manager"""
        base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map")
        
        # Create directory if it doesn't exist
        os.makedirs(base_path, exist_ok=True)
        
        try:
            # Cross-platform way to open folder in file manager
            system = platform.system()
            if system == "Windows":
                os.startfile(base_path)
            elif system == "Darwin":  # macOS
                subprocess.run(["open", base_path])
            else:  # Linux and other Unix-like systems
                subprocess.run(["xdg-open", base_path])
            
            self.status_label.setText("Opened txt_map folder in file manager")
            
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to open folder: {str(e)}")
            self.status_label.setText("Failed to open folder")
    
    def _folder_changed(self, folder_name):
        """Handle folder selection change"""
        if not folder_name:
            return
        
        # Stop live monitoring if active when changing folders
        if self.live_monitor_enabled:
            self._toggle_live_monitor()
        
        self.current_folder = folder_name
        base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map", folder_name)
        
        # Load JSON files from the folder
        json_files = glob.glob(os.path.join(base_path, "*.json"))
        json_files.sort()
        
        self.data_files = json_files
        self._update_file_list()
        
        self.status_label.setText(f"Loaded {len(json_files)} data files")
    
    def _file_selected(self, item):
        """Handle file selection"""
        filename = item.text()
        json_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map", 
                                 self.current_folder, filename)
        
        # Use safe loading method
        data = self._safe_load_json(json_path)
        
        if data is not None:
            self.current_data = data
            self._visualize_data(self.current_data)
            self._update_info_display(self.current_data)
        else:
            # Only show error dialog if not in live monitor mode
            if not self.live_monitor_enabled:
                QMessageBox.warning(self, "Error", f"Failed to load file: {filename}")
            else:
                # In live mode, just update status
                self.status_label.setText(f"Live monitoring: Skipped corrupted file {filename}")
    
    def _file_selected_safe(self, item, show_errors=True):
        """Safe version of file selection for live monitoring"""
        filename = item.text()
        json_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map", 
                                 self.current_folder, filename)
        
        data = self._safe_load_json(json_path)
        
        if data is not None:
            self.current_data = data
            self._visualize_data(self.current_data)
            self._update_info_display(self.current_data)
            return True
        else:
            if show_errors:
                self.status_label.setText(f"Live monitoring: Skipped incomplete file {filename}")
            return False
    
    def _visualize_data(self, data):
        """Visualize the occupancy grid data"""
        grid_width = data['info']['width']
        grid_height = data['info']['height']
        grid_data = data['data']
        
        if grid_width == 0 or grid_height == 0:
            return
        
        cell_size = 800 / max(grid_width, grid_height)
        
        pixmap = QPixmap(800, 800)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        
        for row in range(grid_height):
            real_row = grid_height - 1 - row
            for col in range(grid_width):
                index = real_row * grid_width + col
                if index < len(grid_data):
                    value = grid_data[index]
                    
                    if value == 100:
                        color = QColor(50, 50, 50)
                    elif value == 101:
                        color = QColor(50, 250, 50)
                    elif value == 102:
                        color = QColor(200, 200, 50)
                    elif value == 103:
                        color = QColor(250, 50, 50)
                    elif value == -1:
                        color = QColor('darkgray')
                    elif value == 0:
                        color = QColor('lightgray')
                    else:
                        continue
                    
                    rect = QRectF(col * cell_size, row * cell_size,
                                cell_size, cell_size)
                    painter.fillRect(rect, QBrush(color))
        
        painter.end()
        self.scene.clear()
        self.scene.addPixmap(pixmap)
    
    def _update_info_display(self, data):
        """Update the information display"""
        info = data['info']
        timestamp = data['timestamp']
        
        info_text = f"""
        Timestamp: {timestamp}
        Width: {info['width']}
        Height: {info['height']}
        Resolution: {info['resolution']:.4f}
        Frame ID: {data['header']['frame_id']}
        Data Points: {len(data['data'])}
        """
        
        self.info_label.setText(info_text)
    
    def _toggle_playback(self):
        """Toggle automatic playback"""
        if self.auto_timer.isActive():
            self.auto_timer.stop()
            self.play_btn.setText("Play")
        else:
            try:
                interval = int(self.speed_input.text())
                self.auto_timer.start(interval)
                self.play_btn.setText("Pause")
            except ValueError:
                QMessageBox.warning(self, "Error", "Invalid speed value")
    
    def _stop_playback(self):
        """Stop automatic playback"""
        self.auto_timer.stop()
        self.play_btn.setText("Play")
    
    def _auto_next_frame(self):
        """Automatically advance to next frame"""
        current_row = self.file_list.currentRow()
        if current_row < self.file_list.count() - 1:
            self.file_list.setCurrentRow(current_row + 1)
            self._file_selected(self.file_list.currentItem())
        else:
            self._stop_playback()
    
    def _generate_evaluation_images(self):
        """Generate evaluation images for all data in current folder"""
        if not self.current_folder:
            QMessageBox.warning(self, "Error", "No folder selected")
            return
        
        if not self.data_files:
            QMessageBox.warning(self, "Error", "No data files in current folder")
            return
        
        # Load all data
        data_list = []
        for json_file in self.data_files:
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                    data_list.append(data)
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to load {json_file}: {str(e)}")
                return
        
        # Set output folder
        output_folder = os.path.join(os.path.expanduser("~"), "fasem", "evaluation_images", self.current_folder)
        
        # Start generation thread
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.generate_btn.setEnabled(False)
        self.status_label.setText("Generating images...")
        
        self.image_generator = ImageGeneratorThread(data_list, output_folder)
        self.image_generator.progress_updated.connect(self.progress_bar.setValue)
        self.image_generator.generation_completed.connect(self._generation_completed)
        self.image_generator.error_occurred.connect(self._generation_error)
        self.image_generator.start()
    
    def _generation_completed(self, output_folder):
        """Handle image generation completion"""
        self.progress_bar.setVisible(False)
        self.generate_btn.setEnabled(True)
        self.status_label.setText("Image generation completed")
        
        QMessageBox.information(self, "Success", 
                              f"Evaluation images generated successfully!\nSaved to: {output_folder}")
    
    def _generation_error(self, error_message):
        """Handle image generation error"""
        self.progress_bar.setVisible(False)
        self.generate_btn.setEnabled(True)
        self.status_label.setText("Error during generation")
        
        QMessageBox.critical(self, "Error", f"Image generation failed: {error_message}")
    
    def _toggle_live_monitor(self):
        """Toggle live monitoring of the current folder"""
        if not self.current_folder:
            QMessageBox.warning(self, "Error", "Please select a folder first")
            return
        
        if self.live_monitor_enabled:
            # Stop live monitoring
            self.live_monitor_timer.stop()
            self.live_monitor_enabled = False
            self.live_monitor_btn.setText("Start Live Monitor")
            self.live_status_label.setText("● Monitoring: Off")
            self.live_status_label.setStyleSheet("color: gray;")
            self.status_label.setText("Live monitoring stopped")
            
            # Re-enable manual playback controls
            self.play_btn.setEnabled(True)
            self.stop_btn.setEnabled(True)
            
        else:
            # Start live monitoring
            self.live_monitor_timer.start(1000)  # 1 second interval
            self.live_monitor_enabled = True
            self.live_monitor_btn.setText("Stop Live Monitor")
            self.live_status_label.setText("● Monitoring: On")
            self.live_status_label.setStyleSheet("color: green; font-weight: bold;")
            self.status_label.setText("Live monitoring active - checking for new files every 1 second")
            
            # Disable manual playback controls during live monitoring
            if self.auto_timer.isActive():
                self._stop_playback()
            self.play_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
    
    def _live_monitor_update(self):
        """Update file list and optionally show latest data during live monitoring"""
        if not self.current_folder:
            self._toggle_live_monitor()  # Stop if no folder selected
            return
        
        # Store current file count for comparison
        old_file_count = len(self.data_files)
        old_latest_file = self.data_files[-1] if self.data_files else None
        
        # Refresh file list
        base_path = os.path.join(os.path.expanduser("~"), "fasem", "txt_map", self.current_folder)
        
        if not os.path.exists(base_path):
            self.status_label.setText("Live monitoring: Folder no longer exists")
            self._toggle_live_monitor()
            return
        
        # Load JSON files from the folder and filter out very recent files (might still be writing)
        all_json_files = glob.glob(os.path.join(base_path, "*.json"))
        
        # Filter files by age to avoid files that are currently being written
        # Skip files modified in the last threshold seconds
        current_time = time.time()
        json_files = []
        
        for json_file in all_json_files:
            try:
                file_mtime = os.path.getmtime(json_file)
                if current_time - file_mtime > self.file_age_threshold:
                    json_files.append(json_file)
            except OSError:
                # Skip files that can't be accessed
                continue
        
        json_files.sort()
        
        # Check if there are new files
        if len(json_files) != old_file_count or (json_files and json_files != self.data_files):
            self.data_files = json_files
            self._update_file_list()
            
            new_file_count = len(json_files)
            files_diff = new_file_count - old_file_count
            if files_diff > 0:
                self.status_label.setText(f"Live monitoring: {new_file_count} files (+{files_diff} new)")
            else:
                self.status_label.setText(f"Live monitoring: {new_file_count} files (updated)")
            
            # Auto show latest file if enabled and there are new files
            if (self.auto_latest_checkbox.isChecked() and 
                json_files and 
                (old_latest_file != json_files[-1])):
                
                # Select and display the latest file
                latest_filename = os.path.basename(json_files[-1])
                
                # Find and select the item in the list
                for i in range(self.file_list.count()):
                    if self.file_list.item(i).text() == latest_filename:
                        self.file_list.setCurrentRow(i)
                        # Use safe file selection for live mode
                        success = self._file_selected_safe(self.file_list.item(i), show_errors=False)
                        if success:
                            self.status_label.setText(f"Live monitoring: Showing latest file {latest_filename}")
                        break
        else:
            # Update status to show monitoring is active
            current_time_str = datetime.datetime.now().strftime("%H:%M:%S")
            available_files = len(all_json_files)
            ready_files = len(json_files)
            
            if available_files > ready_files:
                self.status_label.setText(f"Live monitoring: {ready_files}/{available_files} files ready - Last check: {current_time_str}")
            else:
                self.status_label.setText(f"Live monitoring: {ready_files} files - Last check: {current_time_str}")
    
    def _update_file_list(self):
        """Update the file list widget with current data files"""
        self.file_list.clear()
        
        for json_file in self.data_files:
            filename = os.path.basename(json_file)
            item = QListWidgetItem(filename)
            self.file_list.addItem(item)
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Stop all timers before closing
        if self.live_monitor_enabled:
            self.live_monitor_timer.stop()
        if self.auto_timer.isActive():
            self.auto_timer.stop()
        
        # Stop any ongoing image generation
        if hasattr(self, 'image_generator') and self.image_generator.isRunning():
            self.image_generator.terminate()
            self.image_generator.wait()
        
        event.accept()
    
    def _safe_load_json(self, json_path, max_retries=None, retry_delay=None):
        """Safely load JSON file with retry mechanism for concurrent access"""
        if max_retries is None:
            max_retries = self.json_load_retries
        if retry_delay is None:
            retry_delay = self.json_retry_delay
            
        for attempt in range(max_retries):
            try:
                # Check if file exists and has content
                if not os.path.exists(json_path):
                    return None
                
                # Check file size to avoid reading incomplete files
                file_size = os.path.getsize(json_path)
                if file_size == 0:
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    return None
                
                # Check if file is too recent (might still be writing)
                if self.live_monitor_enabled:
                    file_mtime = os.path.getmtime(json_path)
                    current_time = time.time()
                    if current_time - file_mtime < self.file_age_threshold:
                        if attempt < max_retries - 1:
                            time.sleep(retry_delay)
                            continue
                        return None
                
                # Try to read the file
                with open(json_path, 'r') as f:
                    data = json.load(f)
                return data
                
            except json.JSONDecodeError as e:
                # JSON is incomplete or corrupted, likely being written
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    # Log the error but don't show popup in live mode
                    if self.live_monitor_enabled:
                        print(f"Warning: JSON decode error for {os.path.basename(json_path)}: {str(e)}")
                    return None
                    
            except (IOError, OSError) as e:
                # File access error
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    if self.live_monitor_enabled:
                        print(f"Warning: File access error for {os.path.basename(json_path)}: {str(e)}")
                    return None
                    
            except Exception as e:
                # Other unexpected errors
                if self.live_monitor_enabled:
                    print(f"Warning: Unexpected error loading {os.path.basename(json_path)}: {str(e)}")
                return None
        
        return None


def main(args=None):
    app = QApplication(sys.argv)
    window = DataVisualizerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()