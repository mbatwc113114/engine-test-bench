import json
from pathlib import Path

from PyQt5.QtWidgets import QWidget, QFormLayout, QVBoxLayout, QHBoxLayout, QPushButton, QLabel

from ui.widgets import ConfigRow, PinSelector, SensorSelector, RateSelector, SectionHeader


class SetupPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.config_path = Path(__file__).resolve().parents[1] / "config" / "hardware_config.json"
        self.load_defaults()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 10, 20, 10)
        layout.setSpacing(12)

        title = QLabel("SETUP")
        title.setStyleSheet("font-size: 22px; font-weight: bold;")
        layout.addWidget(title)

        self.form = QFormLayout()
        self.form.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(self.form)

        self.build_ui()
        self.load_configuration()

    def load_defaults(self):
        self.default_config = {
            "teensy": {
                "load_cells": {
                    "load_cell_1": {"dt_pin": None, "sck_pin": None},
                    "load_cell_2": {"dt_pin": None, "sck_pin": None},
                },
                "thermocouples": {
                    "tc1": {"module": "MAX6675", "cs_pin": None, "sck_pin": None, "so_pin": None},
                    "tc2": {"module": "MAX6675", "cs_pin": None, "sck_pin": None, "so_pin": None},
                },
                "vibration": {"x": None, "y": None, "z": None},
                "sampling": {"sensor_sample_rate_hz": 100, "telemetry_rate_hz": 20, "logging_rate_hz": 50},
            },
            "esp32": {
                "servo": {"pin": None, "min_us": 1000, "max_us": 2000},
                "telemetry": {"rate_hz": 20},
                "logging": {"rate_hz": 100},
                "udp": {"port": 5000},
            },
        }

    def build_ui(self):
        self.section_teensy = SectionHeader("TEENSY 4.1")
        self.form.addRow(self.section_teensy)

        self.ld1_dt = PinSelector()
        self.ld1_sck = PinSelector()
        self.ld2_dt = PinSelector()
        self.ld2_sck = PinSelector()

        self.form.addRow("Load Cell 1 DT", self.ld1_dt)
        self.form.addRow("Load Cell 1 SCK", self.ld1_sck)
        self.form.addRow("Load Cell 2 DT", self.ld2_dt)
        self.form.addRow("Load Cell 2 SCK", self.ld2_sck)

        self.tc1_module = SensorSelector()
        self.tc1_cs = PinSelector()
        self.tc1_sck = PinSelector()
        self.tc1_so = PinSelector()
        self.tc2_module = SensorSelector()
        self.tc2_cs = PinSelector()
        self.tc2_sck = PinSelector()
        self.tc2_so = PinSelector()

        self.form.addRow("TC1 Module", self.tc1_module)
        self.form.addRow("TC1 CS", self.tc1_cs)
        self.form.addRow("TC1 SCK", self.tc1_sck)
        self.form.addRow("TC1 SO", self.tc1_so)
        self.form.addRow("TC2 Module", self.tc2_module)
        self.form.addRow("TC2 CS", self.tc2_cs)
        self.form.addRow("TC2 SCK", self.tc2_sck)
        self.form.addRow("TC2 SO", self.tc2_so)

        self.vibration_x = PinSelector()
        self.vibration_y = PinSelector()
        self.vibration_z = PinSelector()
        self.form.addRow("Vibration X", self.vibration_x)
        self.form.addRow("Vibration Y", self.vibration_y)
        self.form.addRow("Vibration Z", self.vibration_z)

        self.sensor_rate = RateSelector(1, 5000, 100)
        self.telemetry_rate = RateSelector(1, 1000, 20)
        self.logging_rate = RateSelector(1, 1000, 50)
        self.form.addRow("Sensor Rate", self.sensor_rate)
        self.form.addRow("Telemetry Rate", self.telemetry_rate)
        self.form.addRow("Logging Rate", self.logging_rate)

        self.section_esp32 = SectionHeader("ESP32")
        self.form.addRow(self.section_esp32)

        self.servo_pin = PinSelector()
        self.servo_min_us = RateSelector(500, 3000, 1000)
        self.servo_max_us = RateSelector(500, 3000, 2000)
        self.udp_port = RateSelector(1000, 65535, 5000)
        self.esp_rate = RateSelector(1, 1000, 20)

        self.form.addRow("Servo Pin", self.servo_pin)
        self.form.addRow("Servo Min us", self.servo_min_us)
        self.form.addRow("Servo Max us", self.servo_max_us)
        self.form.addRow("UDP Port", self.udp_port)
        self.form.addRow("ESP32 Rate", self.esp_rate)

        button_row = QHBoxLayout()
        button_row.setSpacing(10)
        self.apply_button = QPushButton("APPLY")
        self.save_button = QPushButton("SAVE")
        self.reset_button = QPushButton("RESET")
        button_row.addWidget(self.apply_button)
        button_row.addWidget(self.save_button)
        button_row.addWidget(self.reset_button)
        layout.addLayout(button_row)

    def load_configuration(self):
        try:
            with open(self.config_path, "r", encoding="utf-8") as handle:
                config = json.load(handle)
        except FileNotFoundError:
            config = self.default_config

        self._populate_from_config(config)

    def _populate_from_config(self, config):
        teensy = config.get("teensy", {})
        esp32 = config.get("esp32", {})

        cell_1 = teensy.get("load_cells", {}).get("load_cell_1", {})
        cell_2 = teensy.get("load_cells", {}).get("load_cell_2", {})
        self.ld1_dt.setCurrentText(cell_1.get("dt_pin") or "None")
        self.ld1_sck.setCurrentText(cell_1.get("sck_pin") or "None")
        self.ld2_dt.setCurrentText(cell_2.get("dt_pin") or "None")
        self.ld2_sck.setCurrentText(cell_2.get("sck_pin") or "None")

        tc1 = teensy.get("thermocouples", {}).get("tc1", {})
        tc2 = teensy.get("thermocouples", {}).get("tc2", {})
        self.tc1_module.setCurrentText(tc1.get("module") or "MAX6675")
        self.tc1_cs.setCurrentText(tc1.get("cs_pin") or "None")
        self.tc1_sck.setCurrentText(tc1.get("sck_pin") or "None")
        self.tc1_so.setCurrentText(tc1.get("so_pin") or "None")
        self.tc2_module.setCurrentText(tc2.get("module") or "MAX6675")
        self.tc2_cs.setCurrentText(tc2.get("cs_pin") or "None")
        self.tc2_sck.setCurrentText(tc2.get("sck_pin") or "None")
        self.tc2_so.setCurrentText(tc2.get("so_pin") or "None")

        vibration = teensy.get("vibration", {})
        self.vibration_x.setCurrentText(vibration.get("x") or "None")
        self.vibration_y.setCurrentText(vibration.get("y") or "None")
        self.vibration_z.setCurrentText(vibration.get("z") or "None")

        sampling = teensy.get("sampling", {})
        self.sensor_rate.setValue(int(sampling.get("sensor_sample_rate_hz") or 100))
        self.telemetry_rate.setValue(int(sampling.get("telemetry_rate_hz") or 20))
        self.logging_rate.setValue(int(sampling.get("logging_rate_hz") or 50))

        servo = esp32.get("servo", {})
        self.servo_pin.setCurrentText(servo.get("pin") or "None")
        self.servo_min_us.setValue(int(servo.get("min_us") or 1000))
        self.servo_max_us.setValue(int(servo.get("max_us") or 2000))

        udp = esp32.get("udp", {})
        self.udp_port.setValue(int(udp.get("port") or 5000))
        self.esp_rate.setValue(int(esp32.get("telemetry", {}).get("rate_hz") or 20))

    def save_configuration(self):
        config = {
            "teensy": {
                "load_cells": {
                    "load_cell_1": {"dt_pin": self.ld1_dt.currentText() if self.ld1_dt.currentText() != "None" else None, "sck_pin": self.ld1_sck.currentText() if self.ld1_sck.currentText() != "None" else None},
                    "load_cell_2": {"dt_pin": self.ld2_dt.currentText() if self.ld2_dt.currentText() != "None" else None, "sck_pin": self.ld2_sck.currentText() if self.ld2_sck.currentText() != "None" else None},
                },
                "thermocouples": {
                    "tc1": {"module": self.tc1_module.currentText(), "cs_pin": self.tc1_cs.currentText() if self.tc1_cs.currentText() != "None" else None, "sck_pin": self.tc1_sck.currentText() if self.tc1_sck.currentText() != "None" else None, "so_pin": self.tc1_so.currentText() if self.tc1_so.currentText() != "None" else None},
                    "tc2": {"module": self.tc2_module.currentText(), "cs_pin": self.tc2_cs.currentText() if self.tc2_cs.currentText() != "None" else None, "sck_pin": self.tc2_sck.currentText() if self.tc2_sck.currentText() != "None" else None, "so_pin": self.tc2_so.currentText() if self.tc2_so.currentText() != "None" else None},
                },
                "vibration": {"x": self.vibration_x.currentText() if self.vibration_x.currentText() != "None" else None, "y": self.vibration_y.currentText() if self.vibration_y.currentText() != "None" else None, "z": self.vibration_z.currentText() if self.vibration_z.currentText() != "None" else None},
                "sampling": {"sensor_sample_rate_hz": self.sensor_rate.value(), "telemetry_rate_hz": self.telemetry_rate.value(), "logging_rate_hz": self.logging_rate.value()},
            },
            "esp32": {
                "servo": {"pin": self.servo_pin.currentText() if self.servo_pin.currentText() != "None" else None, "min_us": self.servo_min_us.value(), "max_us": self.servo_max_us.value()},
                "telemetry": {"rate_hz": self.esp_rate.value()},
                "logging": {"rate_hz": self.logging_rate.value()},
                "udp": {"port": self.udp_port.value()},
            },
        }

        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.config_path, "w", encoding="utf-8") as handle:
            json.dump(config, handle, indent=4)

    def reset_configuration(self):
        self._populate_from_config(self.default_config)
