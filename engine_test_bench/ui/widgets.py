from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QDoubleSpinBox, QFrame, QLabel, QLineEdit, QSpinBox, QWidget, QVBoxLayout, QHBoxLayout


class SectionHeader(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setStyleSheet("font-size: 16px; font-weight: bold; color: #f0f0f0; margin-top: 8px;")


class ConfigRow(QWidget):
    def __init__(self, label_text, widget, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        label = QLabel(label_text)
        label.setMinimumWidth(180)
        label.setStyleSheet("color: #f0f0f0;")
        layout.addWidget(label)
        layout.addWidget(widget, 1)


class PinSelector(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.addItems(["None", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15", "A0", "A1", "A2", "A3"]) 


class SensorSelector(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.addItems(["MAX6675", "MAX31855", "MAX31856", "Load Cell", "Vibration", "Thermocouple"]) 


class RateSelector(QSpinBox):
    def __init__(self, minimum=1, maximum=1000, value=100, parent=None):
        super().__init__(parent)
        self.setRange(minimum, maximum)
        self.setValue(value)


class StatusIndicator(QFrame):
    def __init__(self, text="READY", parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Box)
        self.setStyleSheet("background: #2d2d2d; border: 1px solid #6a6a6a; border-radius: 6px; color: white; padding: 6px;")
        self.setMinimumHeight(36)
        self.setProperty("status", text)
        self.layout = QVBoxLayout(self)
        self.label = QLabel(text)
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)

    def set_status(self, text):
        self.label.setText(text)


class TextField(QLineEdit):
    def __init__(self, default="", parent=None):
        super().__init__(default, parent)
        self.setPlaceholderText(default if default else "value")
