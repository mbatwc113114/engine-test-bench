# analysis_page.py
# Engine Test Bench DAQ - Analysis page
#
# Requires:
#   pip install pyqtgraph pandas
#
# Import:
#   from analysis_page import AnalysisPage
#
# Supported CSV column names are detected automatically.
# Recommended logger columns:
# time, rpm, throttle, thrust, fuel, cht, egt, vib_x, vib_y, vib_z

import os
import pandas as pd
import pyqtgraph as pg

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFileDialog, QComboBox, QGridLayout, QGroupBox
)


class AnalysisPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.df = None
        self.file_path = None
        self.plots = {}

        self.build_ui()

    def build_ui(self):
        main = QVBoxLayout(self)
        main.setContentsMargins(15, 15, 15, 15)

        title = QLabel("EXPERIMENT ANALYSIS")
        title.setStyleSheet("font-size:18px;font-weight:bold;color:#eeeeee;")
        main.addWidget(title)

        controls = QHBoxLayout()

        self.load_btn = QPushButton("LOAD LOG FILE")
        self.load_btn.clicked.connect(self.load_file)

        self.file_label = QLabel("No file loaded")
        self.file_label.setStyleSheet("color:#aaaaaa;")

        self.x_combo = QComboBox()
        self.x_combo.setMinimumWidth(130)

        controls.addWidget(self.load_btn)
        controls.addWidget(self.file_label)
        controls.addStretch()
        controls.addWidget(QLabel("X AXIS:"))
        controls.addWidget(self.x_combo)

        main.addLayout(controls)

        self.grid = QGridLayout()
        self.grid.setSpacing(10)

        # Six analysis plots
        plot_definitions = [
            ("thrust", "THRUST vs TIME", "Thrust"),
            ("rpm", "RPM vs TIME", "RPM"),
            ("throttle", "THROTTLE vs TIME", "Throttle (%)"),
            ("temperature", "CHT / EGT vs TIME", "Temperature (°C)"),
            ("fuel", "FUEL vs TIME", "Fuel"),
            ("vibration", "VIBRATION X / Y / Z", "Acceleration")
        ]

        for index, (key, title_text, ylabel) in enumerate(plot_definitions):
            plot = pg.PlotWidget()
            plot.setBackground("#111111")
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setTitle(title_text, color="#eeeeee")
            plot.setLabel("left", ylabel)
            plot.setLabel("bottom", "Time (s)")

            row = index // 2
            col = index % 2

            self.grid.addWidget(plot, row, col)
            self.plots[key] = plot

        main.addLayout(self.grid)

        self.info = QLabel("Load a CSV log file to display analysis.")
        self.info.setStyleSheet("color:#aaaaaa;")
        main.addWidget(self.info)

    # ------------------------------------------------------------
    # File loading
    # ------------------------------------------------------------
    def load_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load DAQ Log",
            "",
            "CSV Files (*.csv);;Excel Files (*.xlsx);;All Files (*)"
        )

        if not filename:
            return

        try:
            if filename.lower().endswith(".xlsx"):
                df = pd.read_excel(filename)
            else:
                df = pd.read_csv(filename)

            self.load_dataframe(df, filename)

        except Exception as e:
            self.info.setText(f"Could not load file: {e}")

    def load_dataframe(self, df, filename=""):
        self.df = df
        self.file_path = filename

        # Normalize names
        self.df.columns = [
            str(c).strip().lower().replace(" ", "_")
            for c in self.df.columns
        ]

        self.x_combo.clear()

        for col in self.df.columns:
            self.x_combo.addItem(col)

        # Prefer time
        preferred = self.find_column(
            ["time", "timestamp", "time_s", "elapsed_time"]
        )

        if preferred:
            self.x_combo.setCurrentText(preferred)

        self.file_label.setText(
            os.path.basename(filename) if filename else "Data loaded"
        )

        self.update_plots()

        self.x_combo.currentTextChanged.connect(
            self.update_plots
        )

    # ------------------------------------------------------------
    # Column detection
    # ------------------------------------------------------------
    def find_column(self, candidates):
        if self.df is None:
            return None

        for candidate in candidates:
            if candidate in self.df.columns:
                return candidate

        # Partial matching
        for col in self.df.columns:
            for candidate in candidates:
                if candidate in col:
                    return col

        return None

    def numeric(self, column):
        if column is None or column not in self.df.columns:
            return None

        return pd.to_numeric(
            self.df[column], errors="coerce"
        ).fillna(0).to_numpy()

    # ------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------
    def clear_plots(self):
        for plot in self.plots.values():
            plot.clear()

    def update_plots(self):
        if self.df is None:
            return

        self.clear_plots()

        x_col = self.x_combo.currentText()

        if not x_col:
            return

        x = self.numeric(x_col)

        if x is None:
            return

        # THRUST
        thrust = self.find_column([
            "thrust", "thrust_n", "force", "load_cell_1"
        ])

        if thrust:
            self.plots["thrust"].plot(
                x, self.numeric(thrust),
                pen=pg.mkPen(width=2)
            )

        # RPM
        rpm = self.find_column([
            "rpm", "engine_rpm", "speed"
        ])

        if rpm:
            self.plots["rpm"].plot(
                x, self.numeric(rpm),
                pen=pg.mkPen(width=2)
            )

        # THROTTLE
        throttle = self.find_column([
            "throttle", "throttle_percent", "throttle_pct"
        ])

        if throttle:
            self.plots["throttle"].plot(
                x, self.numeric(throttle),
                pen=pg.mkPen(width=2)
            )

        # TEMPERATURE
        cht = self.find_column([
            "cht", "cht_c", "cylinder_head_temperature"
        ])

        egt = self.find_column([
            "egt", "egt_c", "exhaust_gas_temperature"
        ])

        if cht:
            self.plots["temperature"].plot(
                x, self.numeric(cht),
                pen=pg.mkPen(width=2),
                name="CHT"
            )

        if egt:
            self.plots["temperature"].plot(
                x, self.numeric(egt),
                pen=pg.mkPen(width=2),
                name="EGT"
            )

        # FUEL
        fuel = self.find_column([
            "fuel", "fuel_percent", "fuel_pct",
            "fuel_level", "fuel_flow"
        ])

        if fuel:
            self.plots["fuel"].plot(
                x, self.numeric(fuel),
                pen=pg.mkPen(width=2)
            )

        # VIBRATION
        vib_x = self.find_column([
            "vib_x", "vibration_x", "accel_x", "ax"
        ])

        vib_y = self.find_column([
            "vib_y", "vibration_y", "accel_y", "ay"
        ])

        vib_z = self.find_column([
            "vib_z", "vibration_z", "accel_z", "az"
        ])

        if vib_x:
            self.plots["vibration"].plot(
                x, self.numeric(vib_x),
                pen=pg.mkPen(width=2),
                name="X"
            )

        if vib_y:
            self.plots["vibration"].plot(
                x, self.numeric(vib_y),
                pen=pg.mkPen(width=2),
                name="Y"
            )

        if vib_z:
            self.plots["vibration"].plot(
                x, self.numeric(vib_z),
                pen=pg.mkPen(width=2),
                name="Z"
            )

        rows = len(self.df)
        cols = len(self.df.columns)

        self.info.setText(
            f"Loaded {rows:,} samples  |  {cols} channels  |  "
            f"File: {os.path.basename(self.file_path) if self.file_path else 'data'}"
        )
