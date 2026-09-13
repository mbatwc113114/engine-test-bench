# experiment_page.py
# Engine Test Bench DAQ - merged experiment implementation
#
# Keeps ExperimentPage as the public class so dashboard.py can import it.
# Contains:
#   - Discrete throttle profile
#   - Continuous throttle profile
#   - Custom draggable profile editor
#   - Correct throttle/vibration bar mapping
#   - CSV generation/save/load
#   - RUN / STOP
#   - Compact single-column layout
#
# Signals:
#   runExperiment(dict)
#   stopExperiment()
#   throttleCommand(float)

import csv
import os
import math

from PyQt5.QtCore import pyqtSignal, QTimer, Qt, QPointF
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QTableWidget, QTableWidgetItem,
    QDoubleSpinBox, QComboBox, QHeaderView, QMessageBox,
    QGroupBox, QLineEdit, QFileDialog, QAbstractItemView,
    QFrame, QSlider, QSizePolicy
)

# pyqtgraph is used only for the profile preview.
try:
    import pyqtgraph as pg
except ImportError:
    pg = None


BG = "#090909"
PANEL = "#242424"
PANEL2 = "#181818"
TEXT = "#F2F2F2"
MUTED = "#BDBDBD"
BLUE = "#2D7FF9"
GREEN = "#19C85A"
RED = "#C62C45"
YELLOW = "#F2C94C"
GRID = "#343434"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ProfilePlot(pg.PlotWidget if pg else QWidget):
    """Compact interactive throttle profile editor."""

    pointsChanged = pyqtSignal()

    def __init__(self, parent=None):
        if pg:
            super().__init__(parent)
            self.setBackground("#080808")
            self.showGrid(x=True, y=True, alpha=0.18)
            self.setLabel("bottom", "Time", units="s")
            self.setLabel("left", "Throttle", units="%")
            self.getAxis("bottom").setPen("#AAAAAA")
            self.getAxis("left").setPen("#AAAAAA")
            self.getAxis("bottom").setTextPen("#D0D0D0")
            self.getAxis("left").setTextPen("#D0D0D0")
            self.curve = self.plot(
                [], [], pen=pg.mkPen("#38A8FF", width=3)
            )
            self.scatter = pg.ScatterPlotItem(
                [], [], size=10,
                brush=pg.mkBrush("#38A8FF"),
                pen=pg.mkPen("#FFFFFF", width=1)
            )
            self.addItem(self.scatter)
            self.points = []
            self.drag_index = -1
            self.setMouseEnabled(x=True, y=True)
            self.scene().sigMouseClicked.connect(self._mouse_clicked)
            self.scene().sigMouseMoved.connect(self._mouse_moved)
        else:
            super().__init__(parent)
            self.points = []

    def set_limits(self, xmin, xmax, ymin, ymax):
        if not pg:
            return
        if xmax <= xmin:
            xmax = xmin + 1
        if ymax <= ymin:
            ymax = ymin + 1
        self.setXRange(xmin, xmax, padding=0)
        self.setYRange(ymin, ymax, padding=0)

    def set_points(self, points):
        self.points = sorted(
            [(float(x), float(y)) for x, y in points],
            key=lambda p: p[0]
        )
        self.redraw()

    def redraw(self):
        if not pg:
            return
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        self.curve.setData(xs, ys)
        self.scatter.setData(xs, ys)
        self.pointsChanged.emit()

    def _scene_to_data(self, pos):
        vb = self.plotItem.vb
        p = vb.mapSceneToView(pos)
        return float(p.x()), float(p.y())

    def _nearest_point(self, x, y):
        if not self.points:
            return -1
        xr = max(self.viewRange()[0][1] - self.viewRange()[0][0], 1)
        yr = max(self.viewRange()[1][1] - self.viewRange()[1][0], 1)
        best = -1
        score = 1e9
        for i, (px, py) in enumerate(self.points):
            s = ((px-x)/xr)**2 + ((py-y)/yr)**2
            if s < score:
                score = s
                best = i
        return best if score < 0.0025 else -1

    def _mouse_clicked(self, event):
        if not pg or event.button() != Qt.LeftButton:
            return
        x, y = self._scene_to_data(event.scenePos())

        # Double-click = add point.
        if event.double():
            self.points.append((x, y))
            self.points.sort(key=lambda p: p[0])
            self.redraw()
            return

        self.drag_index = self._nearest_point(x, y)

    def _mouse_moved(self, pos):
        if not pg or self.drag_index < 0:
            return
        x, y = self._scene_to_data(pos)
        y = clamp(y, 0, 100)

        # Keep time ordered and prevent a point from crossing neighbours.
        left = self.points[self.drag_index - 1][0] + 0.01 if self.drag_index > 0 else -math.inf
        right = self.points[self.drag_index + 1][0] - 0.01 if self.drag_index < len(self.points)-1 else math.inf
        x = clamp(x, left, right)

        self.points[self.drag_index] = (x, y)
        self.redraw()

    def mouseReleaseEvent(self, event):
        self.drag_index = -1
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Backspace, Qt.Key_Delete):
            if self.drag_index >= 0 and self.drag_index < len(self.points):
                self.points.pop(self.drag_index)
                self.drag_index = -1
                self.redraw()
                return
        super().keyPressEvent(event)


class ExperimentPage(QWidget):
    runExperiment = pyqtSignal(dict)
    stopExperiment = pyqtSignal()
    throttleCommand = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.running = False
        self.current_step = 0
        self.current_profile = []
        self.current_csv = ""
        self.profile_filename = "engine_test_profile.csv"

        self.sequence_timer = QTimer(self)
        self.sequence_timer.timeout.connect(self.next_step)

        self.profile_timer = QTimer(self)
        self.profile_timer.timeout.connect(self.execute_profile_tick)
        self.profile_start_ms = 0
        self.profile_index = 0

        self.build_ui()
        self.make_discrete_profile()
        self.refresh_profile()

    def build_ui(self):
        self.setStyleSheet(f"""
            QWidget {{
                background: {BG};
                color: {TEXT};
                font-family: Arial;
                font-size: 12px;
            }}
            QGroupBox {{
                border: 1px solid #3B3B3B;
                border-radius: 9px;
                margin-top: 9px;
                padding: 8px;
                font-weight: bold;
                color: {TEXT};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                color: {TEXT};
            }}
            QLabel {{ color: {TEXT}; }}
            QLineEdit, QDoubleSpinBox, QComboBox {{
                background: #151515;
                color: #F4F4F4;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
                min-height: 22px;
                selection-background-color: {BLUE};
            }}
            QComboBox QAbstractItemView {{
                background: #202020;
                color: white;
                selection-background-color: {BLUE};
            }}
            QPushButton {{
                background: #555555;
                color: white;
                border: none;
                border-radius: 7px;
                padding: 8px 12px;
                font-weight: bold;
            }}
            QPushButton:hover {{ background: #666666; }}
            QPushButton:pressed {{ background: #333333; }}
            QTableWidget {{
                background: #111111;
                alternate-background-color: #181818;
                color: #F4F4F4;
                gridline-color: #383838;
                border: 1px solid #444;
            }}
            QHeaderView::section {{
                background: #292929;
                color: white;
                padding: 6px;
                border: 1px solid #444;
            }}
        """)

        main = QVBoxLayout(self)
        main.setContentsMargins(14, 10, 14, 10)
        main.setSpacing(7)

        # Header
        header = QHBoxLayout()
        title = QLabel("THROTTLE EXPERIMENT")
        title.setStyleSheet("font-size:18px;font-weight:900;color:#FFFFFF;")
        self.status = QLabel("PROFILE READY")
        self.status.setStyleSheet(
            f"font-weight:bold;color:{GREEN};padding-left:20px;"
        )

        self.run_btn = QPushButton("▶  RUN")
        self.stop_btn = QPushButton("■  STOP")
        self.run_btn.setStyleSheet(
            f"QPushButton{{background:{GREEN};color:white;}}"
        )
        self.stop_btn.setStyleSheet(
            f"QPushButton{{background:{RED};color:white;}}"
        )
        self.run_btn.clicked.connect(self.start_experiment)
        self.stop_btn.clicked.connect(self.stop_experiment)

        header.addWidget(title)
        header.addWidget(self.status)
        header.addStretch()
        header.addWidget(self.run_btn)
        header.addWidget(self.stop_btn)
        main.addLayout(header)

        # Profile mode
        mode_box = QGroupBox("PROFILE MODE")
        mode_row = QHBoxLayout()
        mode_row.setContentsMargins(4, 2, 4, 2)
        mode_row.addWidget(QLabel("Throttle sequence:"))
        self.mode = QComboBox()
        self.mode.addItems(["Discrete", "Continuous", "Custom"])
        self.mode.currentTextChanged.connect(self.mode_changed)
        mode_row.addWidget(self.mode, 1)
        mode_box.setLayout(mode_row)
        main.addWidget(mode_box)

        # Discrete controls
        self.discrete_box = QGroupBox("DISCRETE PROFILE")
        d = QGridLayout()
        d.setHorizontalSpacing(10)
        d.setVerticalSpacing(5)

        self.d_start = self.spin(0, 100, 5, 1)
        self.d_interval = self.spin(0.1, 100, 5, 1)
        self.d_transition = self.spin(0, 3600, 2, 1)
        self.d_hold = self.spin(0.1, 36000, 10, 1)
        self.d_end = self.spin(0, 100, 80, 1)

        fields = [
            ("Start throttle (%)", self.d_start),
            ("Throttle interval (%)", self.d_interval),
            ("Transition time (s)", self.d_transition),
            ("Hold time / interval (s)", self.d_hold),
            ("End throttle (%)", self.d_end),
        ]
        for r, (label, widget) in enumerate(fields):
            d.addWidget(QLabel(label), r, 0)
            d.addWidget(widget, r, 1)

        self.discrete_box.setLayout(d)
        main.addWidget(self.discrete_box)

        # Continuous controls
        self.continuous_box = QGroupBox("CONTINUOUS PROFILE")
        c = QGridLayout()
        self.c_start = self.spin(0, 100, 5, 1)
        self.c_end = self.spin(0, 100, 80, 1)
        self.c_time = self.spin(0.1, 36000, 120, 1)
        c.addWidget(QLabel("Start throttle (%)"), 0, 0)
        c.addWidget(self.c_start, 0, 1)
        c.addWidget(QLabel("End throttle (%)"), 1, 0)
        c.addWidget(self.c_end, 1, 1)
        c.addWidget(QLabel("Experiment time (s)"), 2, 0)
        c.addWidget(self.c_time, 2, 1)
        self.continuous_box.setLayout(c)
        main.addWidget(self.continuous_box)

        # Custom controls
        self.custom_box = QGroupBox("CUSTOM PROFILE EDITOR")
        cc = QGridLayout()

        self.xmin = self.spin(0, 100000, 0, 2)
        self.xmax = self.spin(0.1, 100000, 120, 2)
        self.ymin = self.spin(-100, 100, 0, 1)
        self.ymax = self.spin(-100, 200, 100, 1)

        cc.addWidget(QLabel("X axis min (s)"), 0, 0)
        cc.addWidget(self.xmin, 0, 1)
        cc.addWidget(QLabel("X axis max (s)"), 0, 2)
        cc.addWidget(self.xmax, 0, 3)
        cc.addWidget(QLabel("Y axis min (%)"), 1, 0)
        cc.addWidget(self.ymin, 1, 1)
        cc.addWidget(QLabel("Y axis max (%)"), 1, 2)
        cc.addWidget(self.ymax, 1, 3)

        for w in (self.xmin, self.xmax, self.ymin, self.ymax):
            w.valueChanged.connect(self.axis_changed)

        help_label = QLabel(
            "Double-click = add point   •   Drag = move point   •   "
            "Select point + Delete/Backspace = delete"
        )
        help_label.setStyleSheet(f"color:{MUTED};font-size:11px;")
        cc.addWidget(help_label, 2, 0, 1, 4)
        self.custom_box.setLayout(cc)
        main.addWidget(self.custom_box)

        # Preview below controls, same column.
        preview_box = QGroupBox("THROTTLE VS TIME — PREVIEW")
        pv = QVBoxLayout()
        if pg:
            self.plot = ProfilePlot()
            self.plot.setMinimumHeight(300)
            self.plot.setSizePolicy(
                QSizePolicy.Expanding, QSizePolicy.Expanding
            )
            self.plot.pointsChanged.connect(self.custom_points_changed)
            pv.addWidget(self.plot)
        else:
            self.plot = None
            warning = QLabel("Install pyqtgraph for interactive profile editing.")
            warning.setStyleSheet("color:#FFB74D;")
            pv.addWidget(warning)

        self.profile_info = QLabel("Duration: 0 s   |   Points: 0")
        self.profile_info.setStyleSheet(f"color:{MUTED};")
        pv.addWidget(self.profile_info)
        preview_box.setLayout(pv)
        main.addWidget(preview_box, 1)

        # File + data
        file_box = QGroupBox("PROFILE FILE")
        fb = QHBoxLayout()
        self.file_name = QLineEdit(self.profile_filename)
        self.file_name.setToolTip("CSV filename remembered for this page.")
        generate = QPushButton("GENERATE PROFILE")
        save = QPushButton("SAVE CSV")
        load = QPushButton("LOAD CSV")
        generate.clicked.connect(self.generate_profile)
        save.clicked.connect(self.save_csv)
        load.clicked.connect(self.load_csv)
        fb.addWidget(self.file_name, 1)
        fb.addWidget(generate)
        fb.addWidget(save)
        fb.addWidget(load)
        file_box.setLayout(fb)
        main.addWidget(file_box)

        data_box = QGroupBox("PROFILE DATA")
        dv = QVBoxLayout()
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Time (s)", "Throttle (%)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setMaximumHeight(145)
        dv.addWidget(self.table)
        data_box.setLayout(dv)
        main.addWidget(data_box)

        self.mode_changed("Discrete")

    @staticmethod
    def spin(lo, hi, value, decimals):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(decimals)
        s.setValue(value)
        s.setSingleStep(0.1 if decimals else 1)
        return s

    def mode_changed(self, mode):
        self.discrete_box.setVisible(mode == "Discrete")
        self.continuous_box.setVisible(mode == "Continuous")
        self.custom_box.setVisible(mode == "Custom")

        if mode == "Discrete":
            self.make_discrete_profile()
        elif mode == "Continuous":
            self.make_continuous_profile()
        else:
            if not self.current_profile:
                self.current_profile = [(0.0, 5.0), (120.0, 60.0)]
            self.refresh_profile()

    # ---------------- profile generation ----------------

    def make_discrete_profile(self):
        start = self.d_start.value()
        interval = self.d_interval.value()
        transition = self.d_transition.value()
        hold = self.d_hold.value()
        end = self.d_end.value()

        if interval <= 0:
            interval = 1

        direction = 1 if end >= start else -1
        step = abs(interval) * direction

        levels = [start]
        value = start
        while True:
            nxt = value + step
            if (direction > 0 and nxt >= end) or (direction < 0 and nxt <= end):
                if abs(levels[-1] - end) > 1e-9:
                    levels.append(end)
                break
            levels.append(nxt)
            value = nxt

        points = [(0.0, levels[0])]
        t = 0.0

        for i, level in enumerate(levels):
            if i == 0:
                t += hold
                points.append((t, level))
                continue

            prev = levels[i - 1]

            # Transition to the new throttle.
            if transition > 0:
                points.append((t, prev))
                t += transition
                points.append((t, level))
            else:
                points.append((t, level))

            # Hold each level.
            if i < len(levels) - 1:
                t += hold
                points.append((t, level))

        self.current_profile = self.densify(points, 0.1)
        self.refresh_profile()

    def make_continuous_profile(self):
        start = self.c_start.value()
        end = self.c_end.value()
        duration = self.c_time.value()

        n = max(2, int(duration * 10) + 1)
        self.current_profile = [
            (duration * i / (n - 1),
             start + (end - start) * i / (n - 1))
            for i in range(n)
        ]
        self.refresh_profile()

    @staticmethod
    def densify(points, dt=0.1):
        if not points:
            return []

        out = []
        for i in range(len(points) - 1):
            t0, y0 = points[i]
            t1, y1 = points[i + 1]
            out.append((t0, y0))
            if t1 > t0:
                count = max(1, int((t1 - t0) / dt))
                for k in range(1, count):
                    a = k / count
                    out.append((t0 + (t1-t0)*a, y0 + (y1-y0)*a))
        out.append(points[-1])
        return out

    def generate_profile(self):
        mode = self.mode.currentText()

        if mode == "Discrete":
            self.make_discrete_profile()
        elif mode == "Continuous":
            self.make_continuous_profile()
        else:
            if self.plot:
                self.current_profile = sorted(
                    self.plot.points, key=lambda p: p[0]
                )
            self.refresh_profile()

        self.status.setText("PROFILE READY")
        self.status.setStyleSheet(
            f"font-weight:bold;color:{GREEN};padding-left:20px;"
        )

    def custom_points_changed(self):
        if self.mode.currentText() == "Custom" and self.plot:
            self.current_profile = sorted(
                self.plot.points, key=lambda p: p[0]
            )
            self.refresh_table_only()

    def axis_changed(self):
        if self.plot:
            self.plot.set_limits(
                self.xmin.value(), self.xmax.value(),
                self.ymin.value(), self.ymax.value()
            )

    def refresh_profile(self):
        if self.plot:
            self.plot.set_points(self.current_profile)
            self.plot.set_limits(
                self.xmin.value(), self.xmax.value(),
                self.ymin.value(), self.ymax.value()
            )

        self.refresh_table_only()

    def refresh_table_only(self):
        self.table.setRowCount(0)
        # Keep the table compact for large 1000 Hz profiles.
        shown = self.current_profile
        if len(shown) > 500:
            stride = max(1, len(shown) // 500)
            shown = shown[::stride]

        for t, throttle in shown:
            r = self.table.rowCount()
            self.table.insertRow(r)
            self.table.setItem(r, 0, QTableWidgetItem(f"{t:.3f}"))
            self.table.setItem(r, 1, QTableWidgetItem(f"{throttle:.2f}"))

        duration = self.current_profile[-1][0] if self.current_profile else 0
        self.profile_info.setText(
            f"Duration: {duration:.2f} s   |   "
            f"Points: {len(self.current_profile)}"
        )

    # ---------------- CSV ----------------

    def save_csv(self):
        if not self.current_profile:
            QMessageBox.warning(self, "Profile", "Generate a profile first.")
            return

        name = self.file_name.text().strip() or "engine_test_profile.csv"
        if not name.lower().endswith(".csv"):
            name += ".csv"

        path, _ = QFileDialog.getSaveFileName(
            self, "Save throttle profile", name, "CSV files (*.csv)"
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "throttle_percent"])
                writer.writerows(
                    [f"{t:.6f}", f"{y:.6f}"]
                    for t, y in self.current_profile
                )

            self.current_csv = path
            self.profile_filename = os.path.basename(path)
            self.file_name.setText(self.profile_filename)
            self.status.setText("PROFILE SAVED")
        except Exception as e:
            QMessageBox.critical(self, "Save error", str(e))

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load throttle profile", self.current_csv or "",
            "CSV files (*.csv)"
        )
        if not path:
            return

        points = []
        try:
            with open(path, newline="") as f:
                reader = csv.reader(f)
                rows = list(reader)

            for row in rows[1:] if rows and not self._is_number(rows[0][0]) else rows:
                if len(row) < 2:
                    continue
                try:
                    points.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue

            if not points:
                raise ValueError("No valid time/throttle rows found.")

            self.current_profile = sorted(points)
            self.current_csv = path
            self.profile_filename = os.path.basename(path)
            self.file_name.setText(self.profile_filename)

            if self.plot:
                self.plot.set_points(self.current_profile)

            self.refresh_table_only()
            self.status.setText("PROFILE LOADED")
        except Exception as e:
            QMessageBox.critical(self, "Load error", str(e))

    @staticmethod
    def _is_number(value):
        try:
            float(value)
            return True
        except Exception:
            return False

    # ---------------- experiment execution ----------------

    def start_experiment(self):
        if self.running:
            return

        self.generate_profile()
        if not self.current_profile:
            QMessageBox.warning(self, "No profile", "Generate a profile first.")
            return

        self.running = True
        self.profile_index = 0
        self.current_step = 0

        experiment = {
            "name": self.file_name.text().strip() or "ENGINE_TEST",
            "filename": self.profile_filename,
            "csv_path": self.current_csv,
            "mode": self.mode.currentText(),
            "profile": list(self.current_profile),
            "time_s": [p[0] for p in self.current_profile],
            "throttle_percent": [p[1] for p in self.current_profile],
        }

        self.runExperiment.emit(experiment)

        # Execute using the actual profile timestamps.
        self.profile_start_ms = QTimer().remainingTime()  # harmless placeholder
        self.profile_index = 0
        self._profile_elapsed = 0.0
        self._profile_tick_ms = 50
        self.profile_timer.start(self._profile_tick_ms)

        self.status.setText("RUNNING")
        self.status.setStyleSheet(
            f"font-weight:bold;color:{GREEN};padding-left:20px;"
        )

    def execute_profile_tick(self):
        if not self.running:
            return

        if self.profile_index >= len(self.current_profile):
            self.finish_experiment()
            return

        target_t, throttle = self.current_profile[self.profile_index]
        self.throttleCommand.emit(clamp(throttle, 0, 100))

        self._profile_elapsed += self._profile_tick_ms / 1000.0

        while (
            self.profile_index < len(self.current_profile)
            and self.current_profile[self.profile_index][0] <= self._profile_elapsed
        ):
            self.profile_index += 1

        if self.profile_index >= len(self.current_profile):
            self.finish_experiment()

    def next_step(self):
        # Compatibility with the old ExperimentPage API.
        self.execute_profile_tick()

    def stop_experiment(self):
        self.sequence_timer.stop()
        self.profile_timer.stop()

        was_running = self.running
        self.running = False

        # Safety: always command zero throttle on stop.
        self.throttleCommand.emit(0.0)

        if was_running:
            self.stopExperiment.emit()

        self.status.setText("STOPPED")
        self.status.setStyleSheet(
            f"font-weight:bold;color:{RED};padding-left:20px;"
        )

    def finish_experiment(self):
        self.sequence_timer.stop()
        self.profile_timer.stop()
        self.running = False

        self.throttleCommand.emit(0.0)
        self.status.setText("EXPERIMENT COMPLETE")
        self.status.setStyleSheet(
            f"font-weight:bold;color:{GREEN};padding-left:20px;"
        )


# Compatibility aliases for code that used the older experiment.py.
ExperimentPanel = ExperimentPage
