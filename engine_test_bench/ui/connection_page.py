from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel


class ConnectionPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 10, 20, 10)
        label = QLabel("CONNECTION")
        label.setStyleSheet("font-size: 22px; font-weight: bold;")
        layout.addWidget(label)
        self.status = QLabel("ESP32 UDP connection")
        layout.addWidget(self.status)
