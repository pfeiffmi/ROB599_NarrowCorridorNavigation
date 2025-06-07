import PyQt6.QtCore as QT_core

import numpy as np

class Lidar_Signals(QT_core.QObject):
    # Start Signal
    start = QT_core.pyqtSignal()
    # Lidar Data Signals
    lidar_image = QT_core.pyqtSignal(np.ndarray)
    crash_warning_color = QT_core.pyqtSignal(str)