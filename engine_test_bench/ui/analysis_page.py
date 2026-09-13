from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QComboBox, QPushButton, QHBoxLayout


class AnalysisPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 10, 20, 10)

        title = QLabel("ANALYSIS")
        title.setStyleSheet("font-size: 22px; font-weight: bold;")
        layout.addWidget(title)

        self.experiment_selector = QComboBox()
        self.experiment_selector.addItems(["None", "Test-001", "Test-002"])
        self.channel_selector = QComboBox()
        self.channel_selector.addItems(["RPM", "Thrust", "Fuel", "CHT", "EGT", "Throttle", "Vibration X", "Vibration Y", "Vibration Z"])
        self.x_axis_selector = QComboBox()
        self.x_axis_selector.addItems(["Time", "Sample", "RPM", "Throttle"])
        self.y_axis_selector = QComboBox()
        self.y_axis_selector.addItems(["RPM", "Thrust", "Fuel", "CHT", "EGT", "Throttle"])

        row = QHBoxLayout()
        row.addWidget(self.experiment_selector)
        row.addWidget(self.channel_selector)
        row.addWidget(self.x_axis_selector)
        row.addWidget(self.y_axis_selector)
        layout.addLayout(row)

        self.load_button = QPushButton("LOAD LOG")
        self.export_csv = QPushButton("EXPORT CSV")
        self.export_png = QPushButton("EXPORT PNG")
        bottom = QHBoxLayout()
        bottom.addWidget(self.load_button)
        bottom.addWidget(self.export_csv)
        bottom.addWidget(self.export_png)
        layout.addLayout(bottom)

        self.plot_placeholder = QLabel("Plot area")
        self.plot_placeholder.setStyleSheet("background: #1b1b1b; border: 1px solid #555; min-height: 300px;")
        self.plot_placeholder.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.plot_placeholder)
