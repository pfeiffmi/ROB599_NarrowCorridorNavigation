import PyQt6.QtCore as QT_core

import numpy as np

class Lidar_Signals(QT_core.QObject):
    # Start Signal
    start = QT_core.pyqtSignal()
    # Lidar Image Signal
    lidar_image = QT_core.pyqtSignal(np.ndarray)