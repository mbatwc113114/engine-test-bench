
import sys
import serial
import time
import csv
import struct

from collections import deque

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QSpinBox,
)

from PyQt5.QtCore import QTimer
import pyqtgraph as pg


# ============================================================
# USER SETTINGS
# ============================================================

PORT = "COM6"                 # Change to your ESP32 COM port
BAUD = 921600

CSV_FILE = "rpm_raw_signal.csv"
BIN_FILE = "rpm_raw_signal.bin"

MAX_POINTS = 10000

GUI_UPDATE_MS = 20

# Display only the latest N seconds approximately
DISPLAY_SECONDS = 2.0


# ============================================================
# SERIAL
# ============================================================

try:
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.001
    )

    print("Connected:", PORT)

except Exception as e:

    print("Serial connection failed:")
    print(e)

    sys.exit(1)


# ============================================================
# DATA
# ============================================================

time_data = deque(maxlen=MAX_POINTS)
signal_data = deque(maxlen=MAX_POINTS)

edge_times = deque(maxlen=1000)

last_timestamp_us = None

rising_edges = 0
falling_edges = 0
total_edges = 0

logging_enabled = True


# ============================================================
# FILES
# ============================================================

csv_file = open(
    CSV_FILE,
    "w",
    newline=""
)

csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "timestamp_us",
    "signal"
])


bin_file = open(
    BIN_FILE,
    "wb"
)


# Binary format:

# uint32 timestamp_us
# uint8  signal

BIN_STRUCT = struct.Struct(
    "<IB"
)


# ============================================================
# GUI
# ============================================================

app = QApplication(sys.argv)

window = QWidget()

window.setWindowTitle(
    "ESP32 Engine RPM Raw Signal Monitor - GPIO 10"
)

window.resize(
    1300,
    800
)


main_layout = QVBoxLayout()

window.setLayout(
    main_layout
)


# ============================================================
# TOP INFORMATION BAR
# ============================================================

info_layout = QHBoxLayout()


connection_label = QLabel(
    "● CONNECTED"
)

connection_label.setStyleSheet(
    "font-size: 16px; font-weight: bold;"
)

info_layout.addWidget(
    connection_label
)


edge_label = QLabel(
    "Edges: 0"
)

edge_label.setStyleSheet(
    "font-size: 16px;"
)

info_layout.addWidget(
    edge_label
)


frequency_label = QLabel(
    "Frequency: 0.00 Hz"
)

frequency_label.setStyleSheet(
    "font-size: 16px;"
)

info_layout.addWidget(
    frequency_label
)


period_label = QLabel(
    "Period: 0.00 ms"
)

period_label.setStyleSheet(
    "font-size: 16px;"
)

info_layout.addWidget(
    period_label
)


state_label = QLabel(
    "GPIO 10: UNKNOWN"
)

state_label.setStyleSheet(
    "font-size: 16px; font-weight: bold;"
)

info_layout.addWidget(
    state_label
)


main_layout.addLayout(
    info_layout
)


# ============================================================
# CONTROL BAR
# ============================================================

control_layout = QHBoxLayout()


start_button = QPushButton(
    "START LOGGING"
)

stop_button = QPushButton(
    "STOP LOGGING"
)

clear_button = QPushButton(
    "CLEAR GRAPH"
)


control_layout.addWidget(
    start_button
)

control_layout.addWidget(
    stop_button
)

control_layout.addWidget(
    clear_button
)


control_layout.addWidget(
    QLabel("Display points:")
)


points_spin = QSpinBox()

points_spin.setMinimum(1000)

points_spin.setMaximum(100000)

points_spin.setValue(
    MAX_POINTS
)


control_layout.addWidget(
    points_spin
)


main_layout.addLayout(
    control_layout
)


# ============================================================
# RAW SIGNAL PLOT
# ============================================================

plot = pg.PlotWidget()

plot.setTitle(
    "RAW ENGINE RPM SENSOR SIGNAL — ESP32 GPIO 10"
)

plot.setLabel(
    "left",
    "GPIO 10 State"
)

plot.setLabel(
    "bottom",
    "Time",
    units="s"
)

plot.setYRange(
    -0.2,
    1.2
)

plot.showGrid(
    x=True,
    y=True
)

plot.setMouseEnabled(
    x=True,
    y=False
)

plot.setLimits(
    yMin=-0.2,
    yMax=1.2
)


curve = plot.plot(
    pen=pg.mkPen(
        width=2
    ),
    stepMode=True
)


main_layout.addWidget(
    plot
)


# ============================================================
# SECOND GRAPH - EDGE INTERVAL
# ============================================================

interval_plot = pg.PlotWidget()

interval_plot.setTitle(
    "Time Between Signal Edges"
)

interval_plot.setLabel(
    "left",
    "Interval",
    units="ms"
)

interval_plot.setLabel(
    "bottom",
    "Edge Number"
)

interval_plot.showGrid(
    x=True,
    y=True
)


interval_curve = interval_plot.plot(
    pen=pg.mkPen(
        width=2
    )
)


main_layout.addWidget(
    interval_plot
)


interval_numbers = deque(
    maxlen=1000
)

interval_values = deque(
    maxlen=1000
)


# ============================================================
# LOGGING CONTROL
# ============================================================

def start_logging():

    global logging_enabled

    logging_enabled = True

    print("Logging STARTED")


def stop_logging():

    global logging_enabled

    logging_enabled = False

    print("Logging STOPPED")


def clear_graph():

    time_data.clear()

    signal_data.clear()

    edge_times.clear()

    interval_numbers.clear()

    interval_values.clear()

    curve.clear()

    interval_curve.clear()

    print("Graph cleared")


start_button.clicked.connect(
    start_logging
)

stop_button.clicked.connect(
    stop_logging
)

clear_button.clicked.connect(
    clear_graph
)


# ============================================================
# SERIAL PROCESSING
# ============================================================

def process_serial():

    global last_timestamp_us
    global rising_edges
    global falling_edges
    global total_edges

    while ser.in_waiting:

        try:

            line = ser.readline().decode(
                "ascii",
                errors="ignore"
            ).strip()

        except Exception:

            continue


        if not line:

            continue


        # Ignore ESP32 startup messages

        if not line.startswith(
            "RAW,"
        ):

            continue


        try:

            parts = line.split(",")

            if len(parts) < 3:

                continue


            timestamp_us = int(
                parts[1]
            )

            state = int(
                parts[2]
            )


        except ValueError:

            continue


        # ====================================================
        # STORE DISPLAY DATA
        # ====================================================

        t = (
            timestamp_us /
            1_000_000.0
        )


        time_data.append(t)

        signal_data.append(state)


        # ====================================================
        # EDGE COUNT
        # ====================================================

        total_edges += 1


        if state == 1:

            rising_edges += 1

        else:

            falling_edges += 1


        # ====================================================
        # EDGE INTERVAL
        # ====================================================

        if last_timestamp_us is not None:

            dt_us = (
                timestamp_us -
                last_timestamp_us
            )


            if dt_us > 0:

                dt_ms = (
                    dt_us /
                    1000.0
                )


                interval_numbers.append(
                    len(interval_numbers)
                )

                interval_values.append(
                    dt_ms
                )


                edge_times.append(
                    timestamp_us
                )


        last_timestamp_us = timestamp_us


        # ====================================================
        # FILE LOGGING
        # ====================================================

        if logging_enabled:

            # CSV

            csv_writer.writerow([
                timestamp_us,
                state
            ])


            # Binary

            bin_file.write(
                BIN_STRUCT.pack(
                    timestamp_us,
                    state
                )
            )


    # ========================================================
    # FLUSH FILES
    # ========================================================

    if logging_enabled:

        csv_file.flush()

        bin_file.flush()


# ============================================================
# UPDATE GUI
# ============================================================

def update_gui():

    process_serial()


    # ========================================================
    # RAW SIGNAL
    # ========================================================

    if len(time_data) > 1:

        curve.setData(
            list(time_data),
            list(signal_data)
        )


        # Automatically show latest signal

        latest_time = time_data[-1]

        plot.setXRange(
            max(
                0,
                latest_time -
                DISPLAY_SECONDS
            ),
            latest_time,
            padding=0
        )


    # ========================================================
    # EDGE INTERVAL GRAPH
    # ========================================================

    if len(interval_values) > 1:

        interval_curve.setData(
            list(interval_numbers),
            list(interval_values)
        )


    # ========================================================
    # FREQUENCY
    # ========================================================

    frequency = 0.0

    period_ms = 0.0


    if len(edge_times) >= 2:

        recent_edges = list(
            edge_times
        )[-20:]


        if len(recent_edges) >= 2:

            dt = (
                recent_edges[-1] -
                recent_edges[0]
            )


            if dt > 0:

                number_intervals = (
                    len(recent_edges) - 1
                )


                frequency = (
                    number_intervals *
                    1_000_000.0 /
                    dt
                )


                period_ms = (
                    1000.0 /
                    frequency
                )


    # ========================================================
    # LABELS
    # ========================================================

    edge_label.setText(
        f"Edges: {total_edges}"
    )


    frequency_label.setText(
        f"Frequency: {frequency:.2f} Hz"
    )


    period_label.setText(
        f"Period: {period_ms:.3f} ms"
    )


    if signal_data:

        current_state = signal_data[-1]

        if current_state:

            state_label.setText(
                "GPIO 10: HIGH"
            )

        else:

            state_label.setText(
                "GPIO 10: LOW"
            )


    # ========================================================
    # LOGGING STATUS
    # ========================================================

    if logging_enabled:

        connection_label.setText(
            "● CONNECTED | LOGGING"
        )

    else:

        connection_label.setText(
            "● CONNECTED | STOPPED"
        )


# ============================================================
# TIMER
# ============================================================

timer = QTimer()

timer.timeout.connect(
    update_gui
)

timer.start(
    GUI_UPDATE_MS
)


# ============================================================
# WINDOW CLOSE
# ============================================================

def close_application():

    print(
        "\nClosing..."
    )


    try:

        csv_file.flush()

        csv_file.close()

    except:

        pass


    try:

        bin_file.flush()

        bin_file.close()

    except:

        pass


    try:

        ser.close()

    except:

        pass


    app.quit()


app.aboutToQuit.connect(
    close_application
)


# ============================================================
# START APPLICATION
# ============================================================

window.show()

print(
    "======================================"
)

print(
    "ESP32 RAW RPM SIGNAL MONITOR"
)

print(
    "GPIO 10"
)

print(
    "CSV:",
    CSV_FILE
)

print(
    "BIN:",
    BIN_FILE
)

print(
    "======================================"
)

sys.exit(
    app.exec_()
)
