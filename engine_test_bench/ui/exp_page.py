from PyQt5.QtWidgets import QWidget, QFormLayout, QVBoxLayout, QPushButton, QLabel, QSlider, QSpinBox, QHBoxLayout


class ExpPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 10, 20, 10)

        title = QLabel("EXPERIMENT")
        title.setStyleSheet("font-size: 22px; font-weight: bold;")
        layout.addWidget(title)

        form = QFormLayout()
        self.experiment_name = QLineEdit("Engine Test")
        self.test_id = QLineEdit("TEST-001")
        self.throttle_mode = QComboBox()
        self.throttle_mode.addItems(["MANUAL", "AUTO", "PROFILE"]) 
        self.target_rpm = QSpinBox()
        self.target_rpm.setRange(0, 20000)
        self.target_rpm.setValue(6000)
        self.sample_rate = QSpinBox()
        self.sample_rate.setRange(1, 5000)
        self.sample_rate.setValue(200)
        form.addRow("Experiment Name", self.experiment_name)
        form.addRow("Test ID", self.test_id)
        form.addRow("Throttle Mode", self.throttle_mode)
        form.addRow("Target RPM", self.target_rpm)
        form.addRow("Sample Rate", self.sample_rate)

        self.throttle_slider = QSlider()
        self.throttle_slider.setRange(0, 100)
        self.throttle_slider.setValue(0)
        self.throttle_value = QLabel("0%")
        self.throttle_slider.valueChanged.connect(lambda v: self.throttle_value.setText(f"{v}%"))
        row = QHBoxLayout()
        row.addWidget(self.throttle_slider)
        row.addWidget(self.throttle_value)
        layout.addLayout(form)
        layout.addLayout(row)

        self.arm_button = QPushButton("ARM")
        self.start_button = QPushButton("START TEST")
        self.stop_button = QPushButton("STOP TEST")
        button_row = QHBoxLayout()
        button_row.addWidget(self.arm_button)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        layout.addLayout(button_row)

        self.status_label = QLabel("READY")
        self.status_label.setStyleSheet("font-weight: bold; color: #d0d0d0;")
        layout.addWidget(self.status_label)
