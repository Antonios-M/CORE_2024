import sys
import hashlib
import os
import json
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QGraphicsOpacityEffect,
    QHBoxLayout, QMessageBox, QLineEdit, QListWidget, QSizePolicy, QStackedWidget,
    QFileDialog, QListWidgetItem, QGroupBox, QFormLayout, QComboBox,
    QColorDialog, QDialog, QDialogButtonBox, QButtonGroup, QSlider
)
from PyQt5.QtGui import QFont, QPixmap, QColor, QIcon
from PyQt5.QtCore import (
    Qt, QPropertyAnimation, QTimer, QSequentialAnimationGroup, QEasingCurve, QSize, QPoint
)
import ghhops_server as hs
import rhino3dm  # Ensure rhino3dm is installed if used
import vlc  # Added for video playback

# ----------------------------- Configuration Parameters -----------------------------

# Window Parameters
window_title = "CORE 2024 Robot Manager"
window_width = 960
window_height = 540

# Paths for resources
logo_path = r"D:/CORE24/resources/images/logo.png"  # Update this path accordingly
robots_image_path = r"D:/CORE24/resources/images/"  # Directory containing robot images
database_file = "D:/CORE24/resources/data/users.json"
robots_file = "D:/CORE24/resources/data/robots.json"
demo_video_path = r"D:/CORE24/resources/videos/demo.mp4"  # Added path for demo video

# Colors and Fonts
yes_button_color = "#007AFF"
no_button_color = "#D3D3D3"
hover_pale_red = "#FFA07A"
hover_yes_button_color = "#99CCFF"
hover_pale_blue = "#AFEEEE"
text_color = "#000000"
font_family = "Helvetica"
startup_background_color = "#FFFFFF"

# Font Sizes
course_title_font_size = 24
project_title_font_size = 16
dialog_title_font_size = 18

# Titles for the Startup Screen
course_title = "CORE 2024"
project_title = "Coworking Robotic Assembly"

# Supported Languages
supported_languages = ["English", "Dutch", "Greek"]

# ----------------------------- Utility Functions -----------------------------

def ensure_file_exists(file_path, default_content):
    """Ensure that the specified file exists. If not, create it with default content."""
    if not os.path.exists(file_path):
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as file:
            if isinstance(default_content, dict) or isinstance(default_content, list):
                json.dump(default_content, file, indent=4)
            else:
                file.write(default_content)

# Ensure that user and robot database files exist
ensure_file_exists(database_file, {})
ensure_file_exists(robots_file, [])

# ----------------------------- Robot Class -----------------------------

class Robot:
    """Class to store robot information."""
    def __init__(self, name, base, reference_frame, model_type):
        self.name = name
        self.base = base
        self.reference_frame = reference_frame  # Tuple of three integers
        self.model_type = model_type  # 'UR5' or 'COMAU_NJ60'
        self.image_path = self.get_image_path()
    
    def get_image_path(self):
        """Return the image path based on the model type."""
        if self.model_type == 'UR5':
            path = os.path.join(robots_image_path, "UR5.png")
        elif self.model_type == 'COMAU_NJ60':
            path = os.path.join(robots_image_path, "COMAU_NJ60.png")
        else:
            path = ""
        
        if not os.path.exists(path):
            # Set to a default placeholder image
            return os.path.join(robots_image_path, "placeholder.png")
        return path
    
    def to_dict(self):
        """Convert the Robot instance to a dictionary for JSON serialization."""
        return {
            "name": self.name,
            "base": self.base,
            "reference_frame": self.reference_frame,
            "model_type": self.model_type
        }
    
    @staticmethod
    def from_dict(data):
        """Create a Robot instance from a dictionary."""
        return Robot(
            name=data.get('name', 'Unnamed Robot'),
            base=data.get('base', 0),
            reference_frame=tuple(data.get('reference_frame', [0, 0, 0])),
            model_type=data.get('model_type', 'Unknown')  # Default to 'Unknown' if missing
        )

# ----------------------------- Custom Widgets -----------------------------

class PulsingButton(QPushButton):
    """A QPushButton that pulses and changes color when hovered."""
    def __init__(self, label, color, hover_color=None, is_negative=False, hover_size_increase=False):
        super().__init__(label)
        self.default_color = color
        self.hover_color = hover_color if hover_color else color
        self.is_negative = is_negative
        self.hover_size_increase = hover_size_increase
        # Remove fixed size to allow flexibility
        # self.setFixedSize(200, 50)  # Removed
        # Set minimum size to ensure usability
        self.setMinimumSize(150, 40)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 8px;
                font-family: {font_family};
                font-size: 14px;
            }}
            QPushButton:hover {{
                background-color: {self.hover_color};
            }}
        """)
        self.setFont(QFont(font_family, 12))
        self.original_size = self.size()
        self.hover_animation = None

    def enterEvent(self, event):
        super().enterEvent(event)
        if self.hover_size_increase:
            self.start_size_animation(1.1)
        self.start_pulse()

    def leaveEvent(self, event):
        super().leaveEvent(event)
        if self.hover_size_increase:
            self.start_size_animation(1.0)
        self.stop_pulse()

    def start_pulse(self):
        self.pulse_timer = QTimer(self)
        self.pulse_timer.timeout.connect(self.pulse_effect)
        self.pulse_opacity = 1.0
        self.pulse_direction = -0.05
        self.pulse_timer.start(50)

    def stop_pulse(self):
        if hasattr(self, 'pulse_timer'):
            self.pulse_timer.stop()
            self.setGraphicsEffect(None)

    def pulse_effect(self):
        if self.pulse_opacity <= 0.5 or self.pulse_opacity >= 1.0:
            self.pulse_direction *= -1
        self.pulse_opacity += self.pulse_direction

        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(self.pulse_opacity)
        self.setGraphicsEffect(opacity_effect)

    def start_size_animation(self, scale_factor):
        new_width = self.original_size.width() * scale_factor
        new_height = self.original_size.height() * scale_factor
        new_size = QSize(int(new_width), int(new_height))

        self.hover_animation = QPropertyAnimation(self, b"size")
        self.hover_animation.setDuration(200)
        self.hover_animation.setStartValue(self.size())
        self.hover_animation.setEndValue(new_size)
        self.hover_animation.setEasingCurve(QEasingCurve.OutCubic)
        self.hover_animation.start()

# ----------------------------- Dialogs -----------------------------

class VideoPlayerDialog(QDialog):
    """Dialog to play demo video."""
    def __init__(self, video_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Watch Demo")
        self.setFixedSize(800, 600)
        layout = QVBoxLayout()
        self.instance = vlc.Instance()
        self.media = self.instance.media_new(video_path)
        self.player = self.instance.media_player_new()
        self.player.set_media(self.media)
        self.video_frame = QWidget(self)
        self.video_frame.setMinimumSize(780, 500)
        self.video_frame.setStyleSheet("background-color: black;")
        layout.addWidget(self.video_frame)
        if sys.platform.startswith('linux'):
            self.player.set_xwindow(self.video_frame.winId())
        elif sys.platform == "win32":
            self.player.set_hwnd(self.video_frame.winId())
        elif sys.platform == "darwin":
            self.player.set_nsobject(int(self.video_frame.winId()))
        # Add position slider
        self.position_slider = QSlider(Qt.Horizontal)
        self.position_slider.setRange(0, 1000)
        self.position_slider.sliderMoved.connect(self.set_position)
        layout.addWidget(self.position_slider)
        controls_layout = QHBoxLayout()
        play_button = QPushButton("Play")
        play_button.clicked.connect(self.player.play)
        pause_button = QPushButton("Pause")
        pause_button.clicked.connect(self.player.pause)
        stop_button = QPushButton("Stop")
        stop_button.clicked.connect(self.stop)
        controls_layout.addWidget(play_button)
        controls_layout.addWidget(pause_button)
        controls_layout.addWidget(stop_button)
        layout.addLayout(controls_layout)
        self.setLayout(layout)
        self.timer = QTimer(self)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start()
        self.player.play()

    def update_ui(self):
        try:
            if self.player is not None:
                if self.player.is_playing():
                    media_length = self.player.get_length()
                    if media_length > 0:
                        current_time = self.player.get_time()
                        position = int(current_time * 1000 / media_length)
                        self.position_slider.blockSignals(True)
                        self.position_slider.setValue(position)
                        self.position_slider.blockSignals(False)
                else:
                    # Player is not playing
                    self.position_slider.blockSignals(True)
                    self.position_slider.setValue(0)
                    self.position_slider.blockSignals(False)
        except Exception as e:
            print(f"Exception in update_ui: {e}")

    def set_position(self, position):
        try:
            if self.player is not None:
                media_length = self.player.get_length()
                if media_length > 0:
                    new_time = position * media_length / 1000
                    self.player.set_time(int(new_time))
        except Exception as e:
            print(f"Exception in set_position: {e}")

    def stop(self):
        try:
            if self.player is not None:
                self.player.stop()
                self.position_slider.setValue(0)
        except Exception as e:
            print(f"Exception in stop: {e}")

class OptionsDialog(QDialog):
    """Dialog for changing language and appearance settings."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Options")
        self.setFixedSize(400, 300)
        self.selected_color = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Language Selection
        lang_layout = QHBoxLayout()
        lang_label = QLabel("Select Language:")
        self.lang_combo = QComboBox()
        self.lang_combo.addItems(supported_languages)
        lang_layout.addWidget(lang_label)
        lang_layout.addWidget(self.lang_combo)
        layout.addLayout(lang_layout)

        # Appearance Settings
        appearance_layout = QHBoxLayout()
        appearance_label = QLabel("Background Color:")
        self.color_button = QPushButton("Choose Color")
        self.color_button.clicked.connect(self.choose_color)
        appearance_layout.addWidget(appearance_label)
        appearance_layout.addWidget(self.color_button)
        appearance_layout.setAlignment(Qt.AlignLeft)
        layout.addLayout(appearance_layout)

        # Dialog Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)

    def choose_color(self):
        color = QColorDialog.getColor(initial=QColor(startup_background_color))
        if color.isValid():
            self.selected_color = color.name()
            self.color_button.setStyleSheet(f"background-color: {self.selected_color};")
            self.color_button.setText("")
        else:
            self.selected_color = None

class AddRobotDialog(QDialog):
    """Dialog to add a new robot with selection and data input."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add New Robot")
        self.setFixedSize(700, 500)  # Adjust as needed
        self.selected_model = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Robot Selection Group
        selection_group = QGroupBox("Select Robot Model")
        selection_layout = QHBoxLayout()

        # Initialize QButtonGroup for exclusive selection
        self.button_group = QButtonGroup(self)
        self.button_group.setExclusive(True)

        # Define robot models
        robots = [
            {
                "model_type": "UR5",
                "image_path": os.path.join(robots_image_path, "UR5.png"),
                "properties": "Manufacturer: Universal Robots\nPayload: 5 kg\nReach: 850 mm"
            },
            {
                "model_type": "COMAU_NJ60",
                "image_path": os.path.join(robots_image_path, "COMAU_NJ60.png"),
                "properties": "Manufacturer: COMAU\nPayload: 60 kg\nReach: 2000 mm"
            }
        ]

        # Create buttons for each robot
        for idx, robot in enumerate(robots):
            robot_layout = QVBoxLayout()
            robot_button = QPushButton()
            robot_button.setCheckable(True)
            robot_button.setStyleSheet("""
                QPushButton {
                    border: 2px solid #ccc;
                    border-radius: 8px;
                    padding: 10px;
                }
                QPushButton:checked {
                    border: 2px solid #007AFF;
                }
            """)
            robot_button.setMinimumSize(150, 150)
            robot_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            # Set robot image
            pixmap = QPixmap(robot["image_path"])
            if pixmap.isNull():
                # Placeholder image if the specified image doesn't exist
                pixmap = QPixmap(150, 150)
                pixmap.fill(QColor('gray'))
            robot_button.setIcon(QIcon(pixmap))
            robot_button.setIconSize(QSize(130, 130))
            robot_button.clicked.connect(self.update_selected_model)
            # Add to button group
            self.button_group.addButton(robot_button, id=idx)
            robot_layout.addWidget(robot_button, alignment=Qt.AlignCenter)
            # Robot name label
            name_label = QLabel(robot["model_type"])
            name_label.setFont(QFont(font_family, 12, QFont.Bold))
            name_label.setAlignment(Qt.AlignCenter)
            robot_layout.addWidget(name_label)
            # Robot properties label
            properties_label = QLabel(robot["properties"])
            properties_label.setFont(QFont(font_family, 10))
            properties_label.setAlignment(Qt.AlignCenter)
            properties_label.setWordWrap(True)
            robot_layout.addWidget(properties_label)
            selection_layout.addLayout(robot_layout)

        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)

        # Data Input Group
        data_group = QGroupBox("Enter Robot Details")
        data_layout = QFormLayout()

        self.robot_name_input = QLineEdit()
        self.robot_name_input.setPlaceholderText("Enter Robot Name (e.g., UR5 A)")
        self.robot_name_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        data_layout.addRow("Robot Name:", self.robot_name_input)

        self.robot_base_input = QLineEdit()
        self.robot_base_input.setPlaceholderText("Enter Robot Base (integer)")
        self.robot_base_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        data_layout.addRow("Robot Base:", self.robot_base_input)

        self.robot_ref_frame_input = QLineEdit()
        self.robot_ref_frame_input.setPlaceholderText("Enter Reference Frame (e.g., 0,0,0)")
        self.robot_ref_frame_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        data_layout.addRow("Reference Frame:", self.robot_ref_frame_input)

        data_group.setLayout(data_layout)

        layout.addWidget(data_group)

        # Dialog Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.add_robot)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)

    def update_selected_model(self):
        """Update the selected robot model based on button selection."""
        selected_button = self.button_group.checkedButton()
        if selected_button:
            idx = self.button_group.id(selected_button)
            self.selected_model = ["UR5", "COMAU_NJ60"][idx]
        else:
            self.selected_model = None

    def add_robot(self):
        """Validate inputs and accept the dialog if valid."""
        if not self.selected_model:
            QMessageBox.warning(self, "Selection Required", "Please select a robot model.")
            return

        name = self.robot_name_input.text().strip()
        base = self.robot_base_input.text().strip()
        ref_frame = self.robot_ref_frame_input.text().strip()

        # Validate inputs
        valid = True
        errors = []

        if not name:
            valid = False
            errors.append("Robot Name is required.")
        if not base.isdigit():
            valid = False
            errors.append("Robot Base must be an integer.")
        if not self.validate_ref_frame(ref_frame):
            valid = False
            errors.append("Reference Frame must be in the format x,y,z with integers.")

        if not valid:
            QMessageBox.warning(self, "Invalid Input", "\n".join(errors))
            return

        # Process Reference Frame
        ref_frame_tuple = tuple(map(int, ref_frame.split(',')))

        # Set the dialog's data
        self.robot_data = {
            "name": name,
            "base": int(base),
            "reference_frame": ref_frame_tuple,
            "model_type": self.selected_model
        }

        self.accept()

    def validate_ref_frame(self, ref_frame_str):
        """Validate the reference frame input."""
        try:
            parts = ref_frame_str.split(',')
            if len(parts) != 3:
                return False
            tuple(map(int, parts))  # Ensure all parts are integers
            return True
        except:
            return False

# ----------------------------- Hops Server Integration -----------------------------

# Initialize Hops
hops = hs.Hops()

@hops.component(
    "/robot_a",
    name="Robot A",
    nickname="RA",
    description="Get details of Robot A",
    inputs=[],
    outputs=[
        hs.HopsString("Name", "N", "Name of Robot A"),
        hs.HopsNumber("Base", "B", "Base identifier"),
        hs.HopsPoint("Reference Frame", "RF", "Reference Frame coordinates"),
        hs.HopsString("Model Type", "MT", "Model type")
    ]
)
def get_robot_a():
    """Get details of the first robot (Robot A)."""
    try:
        with open(robots_file, 'r') as file:
            robots_data = json.load(file)
            if len(robots_data) >= 1:
                robot_a = robots_data[0]
                rf = robot_a.get('reference_frame', [0, 0, 0])
                # Ensure reference_frame has exactly 3 coordinates
                if len(rf) != 3:
                    rf = [0, 0, 0]
                return (
                    robot_a.get('name', ''),
                    robot_a.get('base', 0),
                    rhino3dm.Point3d(*rf),
                    robot_a.get('model_type', '')
                )
            else:
                return "", 0, rhino3dm.Point3d(0, 0, 0), ""
    except Exception as e:
        # Return error message in the Name output and default values for others
        return f"Error: {str(e)}", 0, rhino3dm.Point3d(0, 0, 0), ""

@hops.component(
    "/robot_b",
    name="Robot B",
    nickname="RB",
    description="Get details of Robot B",
    inputs=[],
    outputs=[
        hs.HopsString("Name", "N", "Name of Robot B"),
        hs.HopsNumber("Base", "B", "Base identifier"),
        hs.HopsPoint("Reference Frame", "RF", "Reference Frame coordinates"),
        hs.HopsString("Model Type", "MT", "Model type")
    ]
)
def get_robot_b():
    """Get details of the second robot (Robot B)."""
    try:
        with open(robots_file, 'r') as file:
            robots_data = json.load(file)
            if len(robots_data) >= 2:
                robot_b = robots_data[1]
                rf = robot_b.get('reference_frame', [0, 0, 0])
                # Ensure reference_frame has exactly 3 coordinates
                if len(rf) != 3:
                    rf = [0, 0, 0]
                return (
                    robot_b.get('name', ''),
                    robot_b.get('base', 0),
                    rhino3dm.Point3d(*rf),
                    robot_b.get('model_type', '')
                )
            else:
                return "", 0, rhino3dm.Point3d(0, 0, 0), ""
    except Exception as e:
        # Return error message in the Name output and default values for others
        return f"Error: {str(e)}", 0, rhino3dm.Point3d(0, 0, 0), ""

@hops.component(
    "/all_robots",
    name="All Robots",
    nickname="AR",
    description="Get details of all registered robots",
    inputs=[],
    outputs=[
        hs.HopsString("Robots", "R", "Details of all robots", access=hs.HopsParamAccess.TREE)
    ]
)
def get_all_robots():
    """Get details of all registered robots."""
    try:
        with open(robots_file, 'r') as file:
            robots_data = json.load(file)
            # Convert each robot's reference frame to a comma-separated string
            formatted_robots = []
            for robot in robots_data:
                formatted_robot = robot.copy()
                rf = robot.get('reference_frame', [0, 0, 0])
                if len(rf) != 3:
                    rf = [0, 0, 0]
                formatted_robot['reference_frame'] = ",".join(map(str, rf))
                formatted_robots.append(json.dumps(formatted_robot))  # Serialize to JSON string
            return formatted_robots
    except Exception as e:
        # Return a list with an error message
        return [f"Error: {str(e)}"]

# ----------------------------- Robot Manager GUI -----------------------------

class RobotManagerGUI(QWidget):
    """PyQt5 GUI for managing robots."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robot Manager")
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.registered_robots = []
        self.init_ui()
        self.load_registered_robots()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)

        # Robot Manager Title
        robot_manager_label = QLabel("Robot Manager")
        robot_manager_label.setFont(QFont(font_family, 24))
        robot_manager_label.setStyleSheet(f"color: {text_color};")
        robot_manager_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(robot_manager_label)

        # Add Robot Button
        add_robot_button = QPushButton("Add New Robot")
        add_robot_button.setMinimumSize(150, 40)
        add_robot_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        add_robot_button.setFont(QFont(font_family, 12))
        add_robot_button.clicked.connect(self.open_add_robot_dialog)
        layout.addWidget(add_robot_button, alignment=Qt.AlignCenter)

        # Robot List
        self.robot_list_widget = QListWidget()
        self.robot_list_widget.setMinimumWidth(200)
        self.robot_list_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_list_widget.itemSelectionChanged.connect(self.update_robot_preview)
        self.robot_list_widget.setStyleSheet("""
            QListWidget {
                border: none;
                font-size: 14px;
                font-family: Helvetica;
            }
            QListWidget::item {
                padding: 10px;
            }
            QListWidget::item:selected {
                background-color: #E0E0E0;
                color: black;
            }
        """)
        layout.addWidget(self.robot_list_widget)

        # Robot Preview Label
        self.robot_preview_label = QLabel("Robot Preview")
        self.robot_preview_label.setFont(QFont(font_family, 20))
        self.robot_preview_label.setStyleSheet(f"color: {text_color};")
        self.robot_preview_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.robot_preview_label)

        # Robot Preview Image
        self.robot_preview_image = QLabel()
        self.robot_preview_image.setAlignment(Qt.AlignCenter)
        # Remove fixed size to allow scaling
        # self.robot_preview_image.setFixedSize(300, 300)
        self.robot_preview_image.setMinimumSize(200, 200)
        self.robot_preview_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_preview_image.setStyleSheet("border: 1px solid #ccc;")
        layout.addWidget(self.robot_preview_image)

        # Remove Robot Button
        remove_robot_button = PulsingButton("Remove Selected Robot", no_button_color, hover_color=hover_pale_red)
        remove_robot_button.setMinimumSize(150, 40)
        remove_robot_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        remove_robot_button.clicked.connect(self.remove_selected_robot)
        layout.addWidget(remove_robot_button, alignment=Qt.AlignCenter)

        # Back to Main Menu Button
        back_button = PulsingButton("Back to Menu", no_button_color, hover_color=hover_pale_red, is_negative=True)
        back_button.setMinimumSize(150, 40)
        back_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        back_button.clicked.connect(lambda: self.parent().switch_screen("main_menu"))
        layout.addWidget(back_button, alignment=Qt.AlignCenter)

        self.setLayout(layout)

    def open_add_robot_dialog(self):
        """Open the dialog to add a new robot."""
        dialog = AddRobotDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            robot_data = dialog.robot_data
            # Check for unique robot name
            if self.robot_name_exists(robot_data['name']):
                QMessageBox.warning(self, "Duplicate Name", f"The robot name '{robot_data['name']}' is already taken.")
                return
            # Create Robot instance
            new_robot = Robot(
                name=robot_data['name'],
                base=robot_data['base'],
                reference_frame=robot_data['reference_frame'],
                model_type=robot_data['model_type']
            )
            # Add to registered robots
            self.registered_robots.append(new_robot)
            self.save_registered_robots()
            self.update_robot_list()
            QMessageBox.information(self, "Robot Added", f"Robot '{new_robot.name}' added successfully!")

    def remove_selected_robot(self):
        """Remove the selected robot from the list."""
        selected_items = self.robot_list_widget.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "No Selection", "Please select a robot to remove.")
            return
        confirm = QMessageBox.question(
            self, "Confirm Removal",
            "Are you sure you want to remove the selected robot(s)?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            for item in selected_items:
                robot = item.data(Qt.UserRole)
                self.registered_robots.remove(robot)
            self.save_registered_robots()
            self.update_robot_list()
            # Clear image preview if the removed robot was being previewed
            if not self.robot_list_widget.selectedItems():
                self.robot_preview_image.clear()
                self.robot_preview_label.setText("Robot Preview")
            QMessageBox.information(self, "Robot Removed", "Selected robot(s) removed successfully.")

    def update_robot_list(self):
        """Update the list of registered robots."""
        self.robot_list_widget.clear()
        for robot in self.registered_robots:
            item_text = f"{robot.name} | Model: {robot.model_type} | Base: {robot.base} | Ref Frame: {robot.reference_frame}"
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, robot)
            self.robot_list_widget.addItem(item)

    def update_robot_preview(self):
        """Update the robot image preview when a robot is selected."""
        selected_items = self.robot_list_widget.selectedItems()
        if selected_items:
            robot = selected_items[0].data(Qt.UserRole)
            image_path = robot.image_path
            if os.path.exists(image_path):
                pixmap = QPixmap(image_path).scaled(
                    self.robot_preview_image.width(),
                    self.robot_preview_image.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
                self.robot_preview_image.setPixmap(pixmap)
                self.robot_preview_label.setText(f"Robot Preview: {robot.name}")
            else:
                self.robot_preview_image.clear()
                self.robot_preview_label.setText("Robot Preview")
        else:
            self.robot_preview_image.clear()
            self.robot_preview_label.setText("Robot Preview")

    def robot_name_exists(self, name):
        """Check if a robot with the given name already exists."""
        for robot in self.registered_robots:
            if robot.name.lower() == name.lower():
                return True
        return False

    def load_registered_robots(self):
        """Load registered robots from the JSON file."""
        try:
            with open(robots_file, 'r') as file:
                robots_data = json.load(file)
                self.registered_robots = [Robot.from_dict(robot) for robot in robots_data]
                self.update_robot_list()
        except (json.JSONDecodeError, FileNotFoundError) as e:
            QMessageBox.critical(self, "Error", f"Failed to load robots data: {e}")
            self.registered_robots = []

    def save_registered_robots(self):
        """Save registered robots to the JSON file."""
        try:
            with open(robots_file, 'w') as file:
                json.dump([robot.to_dict() for robot in self.registered_robots], file, indent=4)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save robots data: {e}")

    def resizeEvent(self, event):
        """Handle window resize to adjust images."""
        self.update_robot_preview()
        super().resizeEvent(event)

# ----------------------------- Startup Screen -----------------------------

class StartupScreen(QWidget):
    """Startup Screen with logo and animated titles."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Startup")
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        # Logo
        logo_label = QLabel()
        pixmap = QPixmap(logo_path)
        if pixmap.isNull():
            # Placeholder image if the specified image doesn't exist
            pixmap = QPixmap(200, 200)
            pixmap.fill(QColor('gray'))
        logo_label.setPixmap(pixmap.scaled(
            200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        logo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)

        # Course Title
        course_title_label = QLabel(course_title)
        course_title_label.setFont(QFont(font_family, course_title_font_size, QFont.Bold))
        course_title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(course_title_label)

        # Project Title
        project_title_label = QLabel(project_title)
        project_title_label.setFont(QFont(font_family, project_title_font_size))
        project_title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(project_title_label)

        # Proceed Button
        proceed_button = PulsingButton("Proceed", yes_button_color, hover_color=hover_yes_button_color)
        proceed_button.setMinimumSize(150, 40)
        proceed_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        proceed_button.clicked.connect(lambda: self.parent().switch_screen("login"))
        layout.addWidget(proceed_button, alignment=Qt.AlignCenter)

        self.setLayout(layout)

# ----------------------------- Login Screen -----------------------------

class LoginScreen(QWidget):
    """Login Screen."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Login")
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        # Login Title
        login_label = QLabel("Login")
        login_label.setFont(QFont(font_family, 24))
        login_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(login_label)

        # Username Input
        self.username_input = QLineEdit()
        self.username_input.setPlaceholderText("Username")
        self.username_input.setMinimumSize(300, 40)
        self.username_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.username_input, alignment=Qt.AlignCenter)

        # Password Input
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setPlaceholderText("Password")
        self.password_input.setMinimumSize(300, 40)
        self.password_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.password_input, alignment=Qt.AlignCenter)

        # Login Button
        login_button = PulsingButton("Login", yes_button_color, hover_color=hover_yes_button_color)
        login_button.setMinimumSize(150, 40)
        login_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        login_button.clicked.connect(self.handle_login)
        layout.addWidget(login_button, alignment=Qt.AlignCenter)

        # Navigate to Register
        register_layout = QHBoxLayout()
        register_label = QLabel("Don't have an account?")
        register_button = QPushButton("Register")
        register_button.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #007AFF;
                border: none;
                text-decoration: underline;
            }
            QPushButton:hover {
                color: #005BBB;
            }
        """)
        register_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        register_button.clicked.connect(lambda: self.parent().switch_screen("register"))
        register_layout.addWidget(register_label)
        register_layout.addWidget(register_button)
        register_layout.setAlignment(Qt.AlignCenter)
        layout.addLayout(register_layout)

        self.setLayout(layout)

    def handle_login(self):
        """Handle user login."""
        username = self.username_input.text().strip()
        password = self.password_input.text()

        if not username or not password:
            QMessageBox.warning(self, "Login Failed", "Please enter both username and password.")
            return

        # Hash the password for comparison
        hashed_password = hashlib.sha256(password.encode()).hexdigest()

        if self.verify_user(username, hashed_password):
            QMessageBox.information(self, "Login Successful", f"Welcome, {username}!")
            self.parent().current_user = username
            self.parent().switch_screen("main_menu")
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid username or password.")

    def verify_user(self, username, hashed_password):
        """Verify if the username and password match the database."""
        try:
            with open(database_file, 'r') as file:
                users_data = json.load(file)
                return users_data.get(username) == hashed_password
        except (json.JSONDecodeError, FileNotFoundError):
            return False

# ----------------------------- Register Screen -----------------------------

class RegisterScreen(QWidget):
    """Register Screen."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Register")
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        # Register Title
        register_label = QLabel("Register")
        register_label.setFont(QFont(font_family, 24))
        register_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(register_label)

        # Username Input
        self.new_username_input = QLineEdit()
        self.new_username_input.setPlaceholderText("Enter new username")
        self.new_username_input.setMinimumSize(300, 40)
        self.new_username_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.new_username_input, alignment=Qt.AlignCenter)

        # Password Input
        self.new_password_input = QLineEdit()
        self.new_password_input.setEchoMode(QLineEdit.Password)
        self.new_password_input.setPlaceholderText("Enter password")
        self.new_password_input.setMinimumSize(300, 40)
        self.new_password_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.new_password_input, alignment=Qt.AlignCenter)

        # Confirm Password Input
        self.confirm_password_input = QLineEdit()
        self.confirm_password_input.setEchoMode(QLineEdit.Password)
        self.confirm_password_input.setPlaceholderText("Confirm password")
        self.confirm_password_input.setMinimumSize(300, 40)
        self.confirm_password_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.confirm_password_input, alignment=Qt.AlignCenter)

        # Register Button
        register_button = PulsingButton("Register", yes_button_color, hover_color=hover_yes_button_color)
        register_button.setMinimumSize(150, 40)
        register_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        register_button.clicked.connect(self.handle_register)
        layout.addWidget(register_button, alignment=Qt.AlignCenter)

        # Navigate to Login
        login_layout = QHBoxLayout()
        login_label = QLabel("Already have an account?")
        login_button = QPushButton("Login")
        login_button.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #007AFF;
                border: none;
                text-decoration: underline;
            }
            QPushButton:hover {
                color: #005BBB;
            }
        """)
        login_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        login_button.clicked.connect(lambda: self.parent().switch_screen("login"))
        login_layout.addWidget(login_label)
        login_layout.addWidget(login_button)
        login_layout.setAlignment(Qt.AlignCenter)
        layout.addLayout(login_layout)

        self.setLayout(layout)

    def handle_register(self):
        """Handle user registration."""
        username = self.new_username_input.text().strip()
        password = self.new_password_input.text()
        confirm_password = self.confirm_password_input.text()

        if not username or not password or not confirm_password:
            QMessageBox.warning(self, "Registration Failed", "All fields are required.")
            return

        if self.user_exists(username):
            QMessageBox.warning(self, "Registration Failed", "Username already exists.")
        elif password != confirm_password:
            QMessageBox.warning(self, "Registration Failed", "Passwords do not match.")
        elif len(password) < 6:
            QMessageBox.warning(self, "Registration Failed", "Password must be at least 6 characters.")
        else:
            hashed_password = hashlib.sha256(password.encode()).hexdigest()
            self.save_user(username, hashed_password)
            QMessageBox.information(self, "Registration Successful", f"User '{username}' registered successfully!")
            self.parent().current_user = username
            self.parent().switch_screen("main_menu")

    def user_exists(self, username):
        """Check if a user already exists in the database."""
        try:
            with open(database_file, 'r') as file:
                users_data = json.load(file)
                return username in users_data
        except (json.JSONDecodeError, FileNotFoundError):
            return False

    def save_user(self, username, hashed_password):
        """Save a new user to the database."""
        try:
            with open(database_file, 'r') as file:
                users_data = json.load(file)
        except (json.JSONDecodeError, FileNotFoundError):
            users_data = {}

        users_data[username] = hashed_password

        try:
            with open(database_file, 'w') as file:
                json.dump(users_data, file, indent=4)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save user data: {e}")

# ----------------------------- Main Menu -----------------------------

class MainMenu(QWidget):
    """Main Menu Screen."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Main Menu")
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(50, 50, 50, 50)
        layout.setSpacing(20)

        # Main Menu Title
        main_menu_label = QLabel("Main Menu")
        main_menu_label.setFont(QFont(font_family, 28))
        main_menu_label.setStyleSheet(f"color: {text_color};")
        main_menu_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(main_menu_label)

        # Buttons Layout
        buttons_layout = QVBoxLayout()
        buttons_layout.setSpacing(20)

        # Robot Manager Button
        robot_manager_button = PulsingButton("Robot Manager", yes_button_color, hover_color=hover_yes_button_color)
        robot_manager_button.setMinimumSize(150, 40)
        robot_manager_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        robot_manager_button.clicked.connect(lambda: self.parent().switch_screen("robot_manager"))
        buttons_layout.addWidget(robot_manager_button, alignment=Qt.AlignCenter)

        # Watch Demo Button (Modified)
        watch_demo_button = PulsingButton("Watch Demo", yes_button_color, hover_color=hover_yes_button_color)
        watch_demo_button.setMinimumSize(150, 40)
        watch_demo_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        watch_demo_button.clicked.connect(self.watch_demo)  # Connect to watch_demo method
        buttons_layout.addWidget(watch_demo_button, alignment=Qt.AlignCenter)

        # Options Button
        options_button = PulsingButton("Options", yes_button_color, hover_color=hover_yes_button_color)
        options_button.setMinimumSize(150, 40)
        options_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        options_button.clicked.connect(self.open_options)
        buttons_layout.addWidget(options_button, alignment=Qt.AlignCenter)

        # Logout Button
        logout_button = PulsingButton("Logout", no_button_color, hover_color=hover_pale_red, is_negative=True)
        logout_button.setMinimumSize(150, 40)
        logout_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        logout_button.clicked.connect(self.logout_user)
        buttons_layout.addWidget(logout_button, alignment=Qt.AlignCenter)

        layout.addLayout(buttons_layout)
        self.setLayout(layout)

    def watch_demo(self):
        """Open the demo video in a video player dialog."""
        if not os.path.exists(demo_video_path):
            QMessageBox.critical(self, "Error", f"Demo video not found at: {demo_video_path}")
            return
        video_dialog = VideoPlayerDialog(demo_video_path, self)
        video_dialog.exec_()

    def open_options(self):
        """Open the options dialog."""
        options_dialog = OptionsDialog(self)
        if options_dialog.exec_() == QDialog.Accepted:
            # Handle language change if implemented
            selected_language = options_dialog.lang_combo.currentText()
            QMessageBox.information(self, "Language Changed", f"Language set to {selected_language}.\n(Note: Language switching is not fully implemented.)")

            # Handle color change
            if options_dialog.selected_color:
                self.parent().setStyleSheet(f"background-color: {options_dialog.selected_color};")

    def logout_user(self):
        """Logout the current user and return to the login screen."""
        self.parent().current_user = None
        self.parent().switch_screen("login")

# ----------------------------- Application Manager -----------------------------

class AppManager(QStackedWidget):
    """Manages different screens in the application."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle(window_title)
        # Remove fixed size to allow resizing
        # self.setFixedSize(window_width, window_height)
        self.setMinimumSize(800, 600)  # Optional: set a minimum size
        self.current_user = None  # Track the logged-in user

        # Initialize screens
        self.startup_screen = StartupScreen(self)
        self.login_screen = LoginScreen(self)
        self.register_screen = RegisterScreen(self)
        self.main_menu = MainMenu(self)
        self.robot_manager_screen = RobotManagerGUI()

        # Add screens to stacked widget
        self.addWidget(self.startup_screen)         # Index 0
        self.addWidget(self.login_screen)           # Index 1
        self.addWidget(self.register_screen)        # Index 2
        self.addWidget(self.main_menu)              # Index 3
        self.addWidget(self.robot_manager_screen)   # Index 4

        self.setCurrentIndex(0)  # Show startup screen first

    def switch_screen(self, screen_name):
        """Switch to the specified screen."""
        screen_map = {
            "startup": 0,
            "login": 1,
            "register": 2,
            "main_menu": 3,
            "robot_manager": 4,
            # Add other screens here as needed
            # "structure_preview": 5,   # Placeholder indices for other screens
            # "station_preview": 6      # Adjust as per actual implementation
        }

        index = screen_map.get(screen_name)
        if index is not None and index < self.count():
            self.setCurrentIndex(index)
        else:
            QMessageBox.warning(self, "Navigation Error", f"Screen '{screen_name}' does not exist.")

    def open_robot_manager(self):
        """Open the Robot Manager screen."""
        self.robot_manager_screen.update_robot_list()
        self.switch_screen("robot_manager")

    def resizeEvent(self, event):
        """Handle window resize events if necessary."""
        # Optionally, implement global resize handling here
        super().resizeEvent(event)

# ----------------------------- Run Hops Server -----------------------------

def run_hops_server():
    """Run the Hops server."""
    # Start Hops server on a separate thread
    hops.start(port=5000)  # Corrected method from 'serve' to 'start'

# ----------------------------- Main Execution -----------------------------

def main():
    # Start Hops server in a separate thread
    hops_thread = threading.Thread(target=run_hops_server, daemon=True)
    hops_thread.start()

    # Start the PyQt5 GUI
    app = QApplication(sys.argv)
    manager = AppManager()
    manager.show()  # Launches the window in resizable mode
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
