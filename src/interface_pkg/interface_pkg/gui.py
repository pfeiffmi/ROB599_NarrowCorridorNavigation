# Import pyqt modules
import PyQt6.QtWidgets as QT_widgets
import PyQt6.QtGui as QT_gui
import PyQt6.QtCore as QT_core

#
import pyqtgraph as QT_graph
QT_graph.setConfigOptions(useOpenGL=True)

# Import open cv module
import cv2
import numpy as np
import math

# Import system module
import sys
import threading
import time

import rclpy
import rclpy.node
import rclpy.logging
import rclpy.parameter

from interface_pkg.pyqt6_ros2_bridge import PyQt6_ROS2_Bridge
from interface_pkg.action_clients.client_manager import Client_Manager


# MainApp class
class MainApp(QT_widgets.QMainWindow):
    
    # class constructor
    def __init__(self, display_update_ms=100):
        # call the parent constructor
        super().__init__()

        # Initiallize the thread manager
        self.pyqt6_ros2_bridge = PyQt6_ROS2_Bridge()
        # initiallize the camera thread signal settings
        self.pyqt6_ros2_bridge.lidar_signals.start.connect(self.camera_thread_start)
        self.pyqt6_ros2_bridge.lidar_signals.lidar_image.connect(self.saveImage, QT_core.Qt.ConnectionType.BlockingQueuedConnection)
        self.last_image_signal = None
        self.image_unused = None
        self.teleop_enabled = True
        
        # get the full screen sizing
        screen_size = QT_gui.QGuiApplication.primaryScreen().availableGeometry()
        self.full_screen_width = int(0.95*screen_size.width())
        self.full_screen_height = int(0.95*screen_size.height())
        rclpy.logging.get_logger("test").info(f"({self.full_screen_width}, {self.full_screen_height})")
        
        # initiallize the ui
        self.init_ui()

        # define action client manager
        self.client_manager = Client_Manager()
        
        # start the pyqt6_ros2 bridge in the threadpool
        self.threadpool = QT_core.QThreadPool()
        self.threadpool.start(self.pyqt6_ros2_bridge)
        self.threadpool.start(self.client_manager)

        # show the window
        self.show()

        #begin display updater
        self.display_updater = QT_core.QTimer()
        self.display_updater.timeout.connect(self.update_display)
        self.display_updater.start(display_update_ms)

    
    # Method to initiallize the UI
    def init_ui(self):
        # Set the background
        self.setStyleSheet("background-color: #888888;")

        # Set the window size and title
        #self.showMaximized() #self.setWindowState(QT_core.Qt.WindowState.WindowMaximized) #self.setFixedSize(1750,950)
        self.setWindowTitle(f"Teleoperator Interface")
        
        # Get the left and right layouts of the interface
        v_layout_left = self._get_left_layout()
        v_layout_right = self._get_right_layout()

        # Add each vertical layout to the horizontal layout
        h_space1 = QT_widgets.QSpacerItem(
            math.ceil(0.1*self.full_screen_width), 
            1, 
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum, 
            vPolicy = QT_widgets.QSizePolicy.Policy.Expanding
        )
        h_space2 = QT_widgets.QSpacerItem(
            math.ceil(0.2*self.full_screen_width), 
            1, 
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum, 
            vPolicy = QT_widgets.QSizePolicy.Policy.Expanding
        )

        h_space3 = QT_widgets.QSpacerItem(
            math.ceil(0.1*self.full_screen_width), 
            1, 
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum, 
            vPolicy = QT_widgets.QSizePolicy.Policy.Expanding
        )

        # Define the high-level horizontal layout
        h_layout = QT_widgets.QHBoxLayout()
        # Add the widgets to the horizontal layout
        h_layout.addItem(h_space1)
        h_layout.addLayout(v_layout_left)
        h_layout.addItem(h_space2)
        h_layout.addLayout(v_layout_right)
        h_layout.addItem(h_space3)

        # Define the central widget from the highest level layout
        central_widget = QT_widgets.QWidget(self)
        central_widget.setLayout(h_layout)
        
        # Set the central widget
        self.stacked_widget = QT_widgets.QStackedWidget()
        self.stacked_widget.addWidget(central_widget)
        self.setCentralWidget(self.stacked_widget)


    def _get_left_layout(self):
        # Define the left-side vertical layout
        v_layout_left = QT_widgets.QVBoxLayout()

        # Define a spacer for the upper whitespace before camera feed
        v_space1 = QT_widgets.QSpacerItem(
            1, 
            40,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_left.addItem(v_space1)

        # define the label for the camera feed
        self.lidar_label = QT_widgets.QLabel(alignment=QT_core.Qt.AlignmentFlag.AlignCenter)
        self.lidar_label.setText("Lidar View")
        self.lidar_label.setFixedHeight(60)
        self.lidar_label.setStyleSheet(
            "font-family: sans;" \
            "font-size: 40px;"
        )
        v_layout_left.addWidget(self.lidar_label)

        # Define the image view for the camera feed
        self.lidar_view = QT_graph.PlotWidget()
        self.lidar_view.getPlotItem().hideAxis('bottom')
        self.lidar_view.getPlotItem().hideAxis('left')
        self.image_item = QT_graph.ImageItem(axisOrder='row-major')
        self.lidar_view.addItem(self.image_item)
        self.lidar_view.setMouseEnabled(x=False, y=False)
        self.lidar_view.setFixedWidth(math.ceil(0.4*self.full_screen_width))
        v_layout_left.addWidget(self.lidar_view)
        
        # Define a spacer between the camera feed and buttons
        v_space2 = QT_widgets.QSpacerItem(
            1, 
            20,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_left.addItem(v_space2)

        # Define the button to open the sensors
        self.start_action_btn = QT_widgets.QPushButton("Start Action", clicked=self.start_action)
        self.start_action_btn.setEnabled(True)
        self.start_action_btn.setFixedHeight(100)
        self.start_action_btn.setStyleSheet(
            "QPushButton{background-color: green;}" \
            "QPushButton:pressed{background-color: lightgreen;}" \
            "QPushButton:disabled{background-color: gray;}" \
        )
        self.start_action_btn.setFont(QT_gui.QFont("sans", 25))
        v_layout_left.addWidget(self.start_action_btn)

        # Define a spacer between the start and stop buttons
        v_space3 = QT_widgets.QSpacerItem(
            1, 
            10,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_left.addItem(v_space3)

        # Define the button to stop listening to the sensors
        self.stop_action_btn = QT_widgets.QPushButton("Stop Action", clicked=self.stop_action)
        self.stop_action_btn.setEnabled(False)
        self.stop_action_btn.setFixedHeight(100)
        self.stop_action_btn.setStyleSheet(
            "QPushButton{background-color: darkred;}" \
            "QPushButton:pressed{background-color: pink;}" \
            "QPushButton:disabled{background-color: gray;}" \
        )
        self.stop_action_btn.setFont(QT_gui.QFont("sans", 25))
        v_layout_left.addWidget(self.stop_action_btn)

        # Define a spacer for the lower whitespace after buttons (matching upper whitespace size)
        v_space4 = QT_widgets.QSpacerItem(
            1, 
            50,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_left.addItem(v_space4)

        return(v_layout_left)
        
    def _get_right_layout(self):
        # Define the left-side vertical layout
        v_layout_right = QT_widgets.QVBoxLayout()

        # Define a spacer for the upper whitespace before camera feed
        v_space1 = QT_widgets.QSpacerItem(
            1, 
            100,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_right.addItem(v_space1)

        # define the label for the camera feed
        self.teleop_controls_label = QT_widgets.QLabel(alignment=QT_core.Qt.AlignmentFlag.AlignCenter)
        self.teleop_controls_label.setText("Teleoperation Controls")
        self.teleop_controls_label.setFixedHeight(60)
        self.teleop_controls_label.setStyleSheet(
            "font-family: sans;" \
            "font-size: 40px;"
        )
        v_layout_right.addWidget(self.teleop_controls_label, alignment=QT_core.Qt.AlignmentFlag.AlignHCenter)

        # Define the button to open the sensors
        self.forward_teleop_btn = QT_widgets.QPushButton("Move Forward")
        self.forward_teleop_btn.setEnabled(True)
        self.forward_teleop_btn.setFixedHeight(50)
        self.forward_teleop_btn.setStyleSheet(
            "QPushButton{background-color: #6F6F6F; color: #2F2F2F}" \
            "QPushButton:pressed{background-color: #AFAFAF; color: #6F6F6F}" \
            "QPushButton:disabled{background-color: #CFCFCF; color: #AFAFAF}" \
        )
        self.forward_teleop_btn.setFont(QT_gui.QFont("sans", 25))
        v_layout_right.addWidget(self.forward_teleop_btn, alignment=QT_core.Qt.AlignmentFlag.AlignHCenter)

        # Define the layout for the two buttons side-by-side
        h_layout_turn_btns = QT_widgets.QHBoxLayout()
        # Define the button to turn left
        self.turn_left_btn = QT_widgets.QPushButton("Turn Left")
        self.turn_left_btn.setEnabled(True)
        self.turn_left_btn.setFixedHeight(50)
        self.turn_left_btn.setStyleSheet(
            "QPushButton{background-color: #6F6F6F; color: #2F2F2F}" \
            "QPushButton:pressed{background-color: #AFAFAF; color: #6F6F6F}" \
            "QPushButton:disabled{background-color: #CFCFCF; color: #AFAFAF}" \
        )
        self.turn_left_btn.setFont(QT_gui.QFont("sans", 25))
        h_layout_turn_btns.addWidget(self.turn_left_btn, alignment=QT_core.Qt.AlignmentFlag.AlignHCenter)
        # Define the button to turn right
        self.turn_right_btn = QT_widgets.QPushButton("Turn Right")
        self.turn_right_btn.setEnabled(True)
        self.turn_right_btn.setFixedHeight(50)
        self.turn_right_btn.setStyleSheet(
            "QPushButton{background-color: #6F6F6F; color: #2F2F2F}" \
            "QPushButton:pressed{background-color: #AFAFAF; color: #6F6F6F}" \
            "QPushButton:disabled{background-color: #CFCFCF; color: #AFAFAF}" \
        )
        self.turn_right_btn.setFont(QT_gui.QFont("sans", 25))
        h_layout_turn_btns.addWidget(self.turn_right_btn, alignment=QT_core.Qt.AlignmentFlag.AlignHCenter)

        # add layout to the vertical widget
        v_layout_right.addLayout(h_layout_turn_btns)

        # Define the button to open the sensors
        self.backward_teleop_btn = QT_widgets.QPushButton("Move Backward")
        self.backward_teleop_btn.setEnabled(True)
        self.backward_teleop_btn.setFixedHeight(50)
        self.backward_teleop_btn.setStyleSheet(
            "QPushButton{background-color: #6F6F6F; color: #2F2F2F}" \
            "QPushButton:pressed{background-color: #AFAFAF; color: #6F6F6F}" \
            "QPushButton:disabled{background-color: #CFCFCF; color: #AFAFAF}" \
        )
        self.backward_teleop_btn.setFont(QT_gui.QFont("sans", 25))
        v_layout_right.addWidget(self.backward_teleop_btn, alignment=QT_core.Qt.AlignmentFlag.AlignHCenter)

        # Define a spacer between the start and stop buttons
        v_space_bottom = QT_widgets.QSpacerItem(
            1, 
            110,
            hPolicy = QT_widgets.QSizePolicy.Policy.Minimum,
            vPolicy = QT_widgets.QSizePolicy.Policy.Minimum
        )
        v_layout_right.addItem(v_space_bottom)

        return(v_layout_right)


    # Define the slot to set the image to the label
    @QT_core.pyqtSlot()
    def camera_thread_start(self):
        # log a message that the camera thread has started
        rclpy.logging.get_logger("cam-slot").info("Camera thread started!")


    # Define the slot to set the image to the label
    @QT_core.pyqtSlot(np.ndarray)
    def saveImage(self, image):
        self.last_image_signal = image
        self.image_unused = True


    @QT_core.pyqtSlot()
    def update_display(self):
        # Disable constance updates to change all GUI widgets at the same time
        self.setUpdatesEnabled(False)

        # Check for update on lidar image
        rclpy.logging.get_logger("test").info(f"imgae unused: {self.image_unused}")
        if((self.image_unused is not None) and (self.image_unused)):
            rclpy.logging.get_logger("test").info(f"---Change image")
            # update the camera feed on the display
            self.image_unused = False
            self.image_item.setImage(self.last_image_signal, autoLevels=None)
            #rclpy.logging.get_logger("test").info(f"{np.sum(self.last_image_signal)}")
            #filename = 'image.jpg'
            #cv2.imwrite(filename, self.last_image_signal)

        # enable updates to have all widgets update at once
        self.setUpdatesEnabled(True)

        if(self.teleop_enabled):
            if(self.forward_teleop_btn.isDown()):
                turning_speed_mps = 1.0
            elif(self.backward_teleop_btn.isDown()):
                turning_speed_mps = -1.0
            else:
                turning_speed_mps = 0.0

            if(self.turn_left_btn.isDown()):
                turning_speed_ccw = 0.5
            elif(self.turn_right_btn.isDown()):
                turning_speed_ccw = -0.5
            else:
                turning_speed_ccw = 0.0

            if(self.pyqt6_ros2_bridge.teleop_controls_publisher is not None):
                self.pyqt6_ros2_bridge.teleop_controls_publisher.publish_movement(
                    forward_speed_mps = turning_speed_mps, 
                    turning_speed_ccw = turning_speed_ccw
                )
            else:
                rclpy.logging.get_logger("test").info("NOPE!")


    # callback for the start button being pressed
    @QT_core.pyqtSlot()
    def start_action(self):
        # disable the both buttons, until action starts
        self.start_action_btn.setEnabled(False)
        self.stop_action_btn.setEnabled(False)

        # disable teleoperation
        self.teleop_enabled = False
        self.forward_teleop_btn.setEnabled(False)
        self.turn_left_btn.setEnabled(False)
        self.turn_right_btn.setEnabled(False)
        self.backward_teleop_btn.setEnabled(False)

        # call the action
        self.client_manager.run_action = True
        
        while(not self.client_manager.traverse_action_client.is_action_alive):
            time.sleep(0.05)

        # disable the start action button, but enable the stop action button
        self.start_action_btn.setEnabled(False)
        self.stop_action_btn.setEnabled(True)
        

    # callback for the stop button being pressed
    @QT_core.pyqtSlot()
    def stop_action(self):
        # disable the both buttons, until action stops
        self.start_action_btn.setEnabled(False)
        self.stop_action_btn.setEnabled(False)

        # stop the action
        self.client_manager.run_action = False
        
        while(self.client_manager.traverse_action_client.is_action_alive):
            time.sleep(0.05)

        # disable the stop action button, but enable the start action button
        self.start_action_btn.setEnabled(True)
        self.stop_action_btn.setEnabled(False)

        # enable teleoperation
        self.teleop_enabled = True
        self.forward_teleop_btn.setEnabled(True)
        self.turn_left_btn.setEnabled(True)
        self.turn_right_btn.setEnabled(True)
        self.backward_teleop_btn.setEnabled(True)
        


# Execute when running the python script directly
def main():
    # Initiallize the ROS2 client library
    rclpy.init()
    
    # Define the application
    app = QT_widgets.QApplication(sys.argv)

    # Initiallize the window and execute the app
    window = MainApp(display_update_ms=200) 
    sys.exit(app.exec())



# call when file executed directly
if(__name__ == "__main__"):
    main()