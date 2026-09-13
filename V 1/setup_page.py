# setup_page.py
# Engine Test Bench DAQ - Setup page
#
# Import into dashboard.py:
#   from setup_page import SetupPage
#
# Signals:
#   configUploaded(dict) -> emitted when "UPLOAD DAQ CONFIG" is clicked

import json
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QGroupBox, QFormLayout,
    QHBoxLayout, QComboBox, QSpinBox, QPushButton, QLabel,
    QMessageBox, QFileDialog
)


class SetupPage(QWidget):
    configUploaded = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.build_ui()

    def _pin_combo(self, value=0):
        combo = QComboBox()
        combo.addItems([str(i) for i in range(0, 40)])
        idx = combo.findText(str(value))
        if idx >= 0:
            combo.setCurrentIndex(idx)
        combo.setStyleSheet(
            """
            QComboBox {
                background: #111111;
                color: #f2f2f2;
                border: 1px solid #5d5d5d;
                border-radius: 6px;
                padding: 6px 10px;
                min-height: 25px;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox QAbstractItemView {
                background: #1d1d1d;
                color: #f2f2f2;
                selection-background-color: #2f6fd6;
            }
            """
        )
        return combo

    def build_ui(self):
        self.setStyleSheet(
            """
            QWidget {
                background: #1f1f1f;
                color: #f2f2f2;
            }
            QLabel {
                color: #f2f2f2;
                background: transparent;
                font-size: 13px;
                font-weight: bold;
            }
            QGroupBox {
                background: #2b2b2b;
                border: 1px solid #4e4e4e;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                color: #f2f2f2;
                font-size: 13px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #f2f2f2;
            }
            QPushButton {
                border: none;
                border-radius: 8px;
                color: #ffffff;
                padding: 10px 12px;
                font-weight: bold;
                background: #3c3c3c;
            }
            QPushButton:hover {
                background: #4d4d4d;
            }
            """
        )

        main = QVBoxLayout(self)
        main.setContentsMargins(15, 15, 15, 15)
        main.setSpacing(10)

        title = QLabel("DAQ HARDWARE SETUP")
        title.setStyleSheet(
            "font-size:18px; font-weight:bold; color:#eeeeee;"
        )
        main.addWidget(title)

        grid = QGridLayout()
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        def add_group(title_text, fields):
            box = QGroupBox(title_text)
            form = QFormLayout(box)
            form.setContentsMargins(12, 12, 12, 12)
            form.setHorizontalSpacing(12)
            form.setVerticalSpacing(10)
            for label_text, widget in fields:
                label = QLabel(label_text)
                label.setStyleSheet("color:#f2f2f2; font-weight:bold;")
                form.addRow(label, widget)
            return box

        # Load cell 1
        self.lc1_dt = self._pin_combo(2)
        self.lc1_sck = self._pin_combo(3)
        lc1 = add_group("LOAD CELL 1", [
            ("DATA PIN", self.lc1_dt),
            ("CLOCK PIN", self.lc1_sck),
        ])

        # Load cell 2
        self.lc2_dt = self._pin_combo(4)
        self.lc2_sck = self._pin_combo(5)
        lc2 = add_group("LOAD CELL 2", [
            ("DATA PIN", self.lc2_dt),
            ("CLOCK PIN", self.lc2_sck),
        ])

        # Thermocouple 1
        self.tc1_cs = self._pin_combo(10)
        self.tc1_sck = self._pin_combo(13)
        self.tc1_miso = self._pin_combo(12)
        self.tc1_mosi = self._pin_combo(11)
        tc1 = add_group("THERMOCOUPLE 1", [
            ("CS PIN", self.tc1_cs),
            ("SCK PIN", self.tc1_sck),
            ("MISO PIN", self.tc1_miso),
            ("MOSI PIN", self.tc1_mosi),
        ])

        # Thermocouple 2
        self.tc2_cs = self._pin_combo(9)
        self.tc2_sck = self._pin_combo(13)
        self.tc2_miso = self._pin_combo(12)
        self.tc2_mosi = self._pin_combo(11)
        tc2 = add_group("THERMOCOUPLE 2", [
            ("CS PIN", self.tc2_cs),
            ("SCK PIN", self.tc2_sck),
            ("MISO PIN", self.tc2_miso),
            ("MOSI PIN", self.tc2_mosi),
        ])

        # Engine / actuator
        self.rpm_pin = self._pin_combo(27)
        self.servo_pin = self._pin_combo(14)
        engine = add_group("ENGINE / ACTUATOR", [
            ("RPM SIGNAL PIN", self.rpm_pin),
            ("THROTTLE SERVO PIN", self.servo_pin),
        ])

        # DAQ settings
        self.sample_rate = QSpinBox()
        self.sample_rate.setRange(1, 50000)
        self.sample_rate.setValue(1000)
        self.sample_rate.setStyleSheet(
            """
            QSpinBox {
                background: #111111;
                color: #f2f2f2;
                border: 1px solid #5d5d5d;
                border-radius: 6px;
                padding: 6px 10px;
                min-height: 25px;
            }
            """
        )

        self.udp_port = QSpinBox()
        self.udp_port.setRange(1, 65535)
        self.udp_port.setValue(4210)
        self.udp_port.setStyleSheet(self.sample_rate.styleSheet())

        daq = add_group("DAQ SETTINGS", [
            ("SAMPLE RATE", self.sample_rate),
            ("UDP PORT", self.udp_port),
        ])

        grid.addWidget(lc1, 0, 0)
        grid.addWidget(lc2, 0, 1)
        grid.addWidget(tc1, 1, 0)
        grid.addWidget(tc2, 1, 1)
        grid.addWidget(engine, 2, 0)
        grid.addWidget(daq, 2, 1)

        main.addLayout(grid)

        buttons = QHBoxLayout()
        buttons.setSpacing(12)

        self.upload_btn = QPushButton("↑ UPLOAD DAQ CONFIG")
        self.upload_btn.setStyleSheet(
            "background:#2377ff; color:white;"
        )
        self.upload_btn.clicked.connect(self.upload_config)

        self.save_btn = QPushButton("▣ SAVE CONFIG")
        self.save_btn.setStyleSheet(
            "background:#2a8a4e; color:white;"
        )
        self.save_btn.clicked.connect(self.save_config)

        self.load_btn = QPushButton("▰ LOAD CONFIG")
        self.load_btn.setStyleSheet(
            "background:#6e6e6e; color:white;"
        )
        self.load_btn.clicked.connect(self.load_config)

        buttons.addWidget(self.upload_btn)
        buttons.addWidget(self.save_btn)
        buttons.addWidget(self.load_btn)
        main.addLayout(buttons)

        self.status = QLabel("Configuration ready.")
        self.status.setStyleSheet("color:#b3b3b3; margin-top:8px;")
        main.addWidget(self.status)

    def _value(self, widget):
        if hasattr(widget, "currentText"):
            return int(widget.currentText())
        return int(widget.value())

    def get_config(self):
        return {
            "load_cell_1": {
                "dt": self._value(self.lc1_dt),
                "sck": self._value(self.lc1_sck),
            },
            "load_cell_2": {
                "dt": self._value(self.lc2_dt),
                "sck": self._value(self.lc2_sck),
            },
            "thermocouple_1": {
                "cs": self._value(self.tc1_cs),
                "sck": self._value(self.tc1_sck),
                "miso": self._value(self.tc1_miso),
                "mosi": self._value(self.tc1_mosi),
            },
            "thermocouple_2": {
                "cs": self._value(self.tc2_cs),
                "sck": self._value(self.tc2_sck),
                "miso": self._value(self.tc2_miso),
                "mosi": self._value(self.tc2_mosi),
            },
            "rpm": {
                "pin": self._value(self.rpm_pin),
            },
            "servo": {
                "pin": self._value(self.servo_pin),
                "min_pwm_us": 1000,
                "max_pwm_us": 2000,
            },
            "daq": {
                "sample_rate": self.sample_rate.value(),
                "udp_port": self.udp_port.value(),
            },
        }

    def upload_config(self):
        try:
            cfg = self.get_config()
            self.configUploaded.emit(cfg)
            self.status.setText("DAQ configuration uploaded.")
            QMessageBox.information(self, "DAQ", "DAQ configuration uploaded.")
        except Exception as e:
            QMessageBox.warning(self, "Config Error", str(e))

    def save_config(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Save DAQ Config",
            "daq_config.json",
            "JSON (*.json)"
        )

        if not filepath:
            return

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(self.get_config(), f, indent=4)
            self.status.setText(f"Saved config: {filepath}")
        except Exception as e:
            QMessageBox.warning(self, "Save Error", str(e))

    def load_config(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "Load DAQ Config",
            "",
            "JSON (*.json)"
        )

        if not filepath:
            return

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                cfg = json.load(f)

            self.apply_config(cfg)
            self.status.setText(f"Loaded config: {filepath}")
        except Exception as e:
            QMessageBox.warning(self, "Load Error", str(e))

    def apply_config(self, cfg):
        def set_combo(combo, value):
            idx = combo.findText(str(value))
            if idx >= 0:
                combo.setCurrentIndex(idx)

        if "load_cell_1" in cfg:
            set_combo(self.lc1_dt, cfg["load_cell_1"].get("dt", 2))
            set_combo(self.lc1_sck, cfg["load_cell_1"].get("sck", 3))

        if "load_cell_2" in cfg:
            set_combo(self.lc2_dt, cfg["load_cell_2"].get("dt", 4))
            set_combo(self.lc2_sck, cfg["load_cell_2"].get("sck", 5))

        if "thermocouple_1" in cfg:
            set_combo(self.tc1_cs, cfg["thermocouple_1"].get("cs", 10))
            set_combo(self.tc1_sck, cfg["thermocouple_1"].get("sck", 13))
            set_combo(self.tc1_miso, cfg["thermocouple_1"].get("miso", 12))
            set_combo(self.tc1_mosi, cfg["thermocouple_1"].get("mosi", 11))

        if "thermocouple_2" in cfg:
            set_combo(self.tc2_cs, cfg["thermocouple_2"].get("cs", 9))
            set_combo(self.tc2_sck, cfg["thermocouple_2"].get("sck", 13))
            set_combo(self.tc2_miso, cfg["thermocouple_2"].get("miso", 12))
            set_combo(self.tc2_mosi, cfg["thermocouple_2"].get("mosi", 11))

        if "rpm" in cfg:
            set_combo(self.rpm_pin, cfg["rpm"].get("pin", 27))

        if "servo" in cfg:
            set_combo(self.servo_pin, cfg["servo"].get("pin", 14))

        if "daq" in cfg:
            self.sample_rate.setValue(int(cfg["daq"].get("sample_rate", 1000)))
            self.udp_port.setValue(int(cfg["daq"].get("udp_port", 4210)))
