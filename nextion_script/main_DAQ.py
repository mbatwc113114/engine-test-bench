"""
ENGINE TEST BENCH DAQ - SINGLE FILE V1 + DASHBOARD

This is the merged V1 DAQ application:
- 1920x1080 dashboard template and live gauges
- ESP32 UDP/mDNS communication
- USB/Serial Teensy DAQ input at 921600 baud
- Teensy AA55 binary acceleration packets + CRC16-CCITT
- 2 kHz vibration acquisition / 10 s buffer
- load-cell and MAX31856 text parsing
- Teensy SD LOGSTART / LOGSTOP commands
- Teensy SD binary download
- live vibration bars
- Setup, Experiment and Analysis pages
- Experiment profile communication
- CSV/XLSX analysis

USB/Serial default:
    COM6 @ 921600
Change SERIAL_PORT below if your Teensy appears on another COM port.
"""

import sys
import os
import json
import math
import socket
import threading
import time
import ipaddress
import csv
from collections import deque

import numpy as np
import pandas as pd
import serial
import pyqtgraph as pg

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import (
    Qt, QTimer, QRectF, QPointF, pyqtSignal, QObject, QThread
)
from PyQt5.QtGui import (
    QPixmap, QPainter, QPen, QBrush, QColor, QFont, QTransform, QDoubleValidator
)
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox,
    QStackedWidget, QVBoxLayout, QHBoxLayout, QMessageBox,
    QSizePolicy, QFileDialog, QGridLayout, QGroupBox, QFormLayout,
    QSpinBox, QTableWidget, QTableWidgetItem, QDoubleSpinBox,
    QHeaderView, QLineEdit, QAbstractItemView, QFrame, QSlider,
    QDialog, QListWidget, QProgressBar, QTextEdit, QDialogButtonBox
)

try:
    from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
except ImportError:
    Zeroconf = None
    ServiceBrowser = None
    ServiceListener = object



# ============================================================
# USER CONFIGURATION
# ============================================================

SERIAL_PORT = "COM6"
BAUD_RATE = 921600

# ------------------------------------------------------------
# TEENSY SD LOG COMMANDS
# ------------------------------------------------------------
# Teensy accepts: LOGSTART <filename> and LOGSTOP
LOG_FILENAME_DEFAULT = "test01"
SD_DOWNLOAD_DEFAULT = "test01.bin"

# Teensy SD transfer protocol (matches Teensy_DAQ_SD_Update_v2.ino)
# ASCII control lines:
#   SD_LIST_BEGIN / FILE <name> <size> / SD_LIST_END
#   SD_DOWNLOAD_BEGIN <name> <size>
#   SD_DOWNLOAD_END <name> <size> <crc16_hex>
# Binary data frame:
#   D5 A5 TYPE OFFSET[4] LENGTH[2] DATA CRC16[2]
SD_DOWNLOAD_SYNC = b"\xD5\xA5"
SD_DOWNLOAD_TYPE_DATA = 0x01
SD_DOWNLOAD_HEADER_SIZE = 9
SD_DOWNLOAD_CRC_SIZE = 2
SD_DOWNLOAD_MAX_CHUNK = 1024

ACC_SAMPLE_RATE_HZ = 2000
PLOT_WINDOW_SECONDS = 10

# ------------------------------------------------------------
# DISPLAY SMOOTHING
# ------------------------------------------------------------
# This only affects the on-screen curves. Raw received samples
# remain unchanged in memory / logging.
DISPLAY_SMOOTHING = False
SMOOTHING_WINDOW = 5

# ------------------------------------------------------------
# REAL-TIME KALMAN FILTER (DISPLAY ONLY)
# ------------------------------------------------------------
# Disabled by default because vibration measurements can contain
# real high-frequency content that a filter may suppress.
# Enable only if the raw ADXL335 display is visibly noisy.
KALMAN_FILTER_ENABLED = True
KALMAN_PROCESS_NOISE = 0.02
KALMAN_MEASUREMENT_NOISE = 0.10

# GUI / display timing
PLOT_UPDATE_HZ = 100
DISPLAY_DELAY_SECONDS = 0.0

# Do not start the visual playback immediately.
# First acquire this much data, then play it continuously
# behind the live acquisition by DISPLAY_DELAY_SECONDS.
DISPLAY_PRELOAD_SECONDS = 0.0

# Display window
PLOT_WINDOW_SECONDS = 10.0

GUI_UPDATE_HZ = PLOT_UPDATE_HZ
MAX_BUFFER_SIZE = int(ACC_SAMPLE_RATE_HZ * PLOT_WINDOW_SECONDS)

# ADXL335 calibration - MUST MATCH TEENSY
ADXL_OFFSET_X_V = 1.65
ADXL_OFFSET_Y_V = 1.65
ADXL_OFFSET_Z_V = 1.65

ADXL_SENS_X_V_PER_G = 0.330
ADXL_SENS_Y_V_PER_G = 0.330
ADXL_SENS_Z_V_PER_G = 0.330

ADC_MAX = 4095.0
ADC_REF_V = 3.3

# Current Teensy code sends load-cell values in grams.
LOAD_UNIT = "g"
TEMP_UNIT = "°C"


# ============================================================
# TEENSY BINARY TELEMETRY PROTOCOL
#
# AA 55 TYPE COUNT_L COUNT_H PAYLOAD CRC_L CRC_H
#
# TYPE = 0x01
# Each acceleration sample =:
#   uint16 X
#   uint16 Y
#   uint16 Z
#   = 6 bytes
#
# Current Teensy:
# 2000 Hz / 20 samples per packet = 100 packets/s
# ============================================================

SYNC1 = 0xAA
SYNC2 = 0x55
PACKET_TYPE_ACCEL = 0x01

HEADER_SIZE = 5
CRC_SIZE = 2


# ============================================================
# DATA
# ============================================================

time_data = deque(maxlen=MAX_BUFFER_SIZE)
vib_x = deque(maxlen=MAX_BUFFER_SIZE)
vib_y = deque(maxlen=MAX_BUFFER_SIZE)
vib_z = deque(maxlen=MAX_BUFFER_SIZE)

load1 = 0.0
load2 = 0.0
temp1 = float("nan")
temp2 = float("nan")

rpm = float("nan")

serial_connected = False
running = True

total_samples = 0
good_packets = 0
bad_packets = 0

data_lock = threading.Lock()

serial_command_lock = threading.Lock()
serial_command_serial = None

# SD download state. The serial reader writes the incoming file directly
# to the PC path selected in the GUI, so large files are never held in RAM.
download_lock = threading.Lock()
download_active = False
download_file_handle = None
download_target_path = ""
download_expected_size = 0
download_received_size = 0
download_error = ""
download_completed = False
download_filename = ""
download_last_status = ""
# Once BEGIN is received, the serial reader treats the stream as a
# dedicated SD-download stream until END/ERROR.  This prevents arbitrary
# file bytes from ever being mistaken for telemetry/text framing.
download_stream_mode = False
download_crc16 = 0xFFFF
download_end_crc16 = None

# SD directory listing state.
sd_list_lock = threading.Lock()
sd_list_files = []
sd_list_active = False
sd_list_completed = False
sd_list_error = ""
sd_list_count = 0

# Teensy logging state reported by the firmware.
logging_active = False
active_log_filename = ""
log_record_count = 0
teensy_text_log = deque(maxlen=300)

# Wall-clock time corresponding to sample 0.
# Samples themselves are placed uniformly at ACC_SAMPLE_RATE_HZ.
stream_start_wall_time = None

# ------------------------------------------------------------
# DISPLAY PLAYHEAD
# ------------------------------------------------------------
# This is deliberately separate from the latest received sample.
# The GUI moves this playhead at 60 Hz, so the graph scrolls
# smoothly instead of jumping whenever a 20-sample USB packet arrives.
display_playhead_time = None
display_started = False
last_gui_wall_time = None

# ------------------------------------------------------------
# CONTINUOUS ACQUISITION CLOCK
# ------------------------------------------------------------
# The Teensy sends acceleration in packets. The current packet is
# 20 samples at 2 kHz = 100 packets/s. Packet arrival itself is
# therefore discrete.  These variables let the GUI interpolate
# the acquisition clock between packet arrivals.
last_packet_arrival_wall_time = None
latest_sample_time = None
last_packet_sample_count = 0



# ============================================================
# CRC16-CCITT
# Must exactly match the Teensy implementation.
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def crc16_update(crc: int, data: bytes) -> int:
    """Continue CRC16-CCITT from a previous CRC value."""
    crc &= 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ============================================================
# ADC -> g
# ============================================================

def adc_to_g(raw, offset_v, sensitivity_v_per_g):
    voltage = (raw * ADC_REF_V) / ADC_MAX

    if sensitivity_v_per_g == 0:
        return 0.0

    return (voltage - offset_v) / sensitivity_v_per_g


# ============================================================
# PARSE HUMAN-READABLE TEXT FROM TEENSY
#
# The Teensy currently prints:
#
# LOAD CELL #1 - 10 KG
# Weight: xxxx g
#
# LOAD CELL #2 - 1 KG
# Weight: xxxx g
#
# MAX31856 #1
# Thermocouple: xxxx °C
#
# MAX31856 #2
# Thermocouple: xxxx °C
# ============================================================

text_buffer = ""
text_section = ""


def process_text_line(line):
    """Parse normal Teensy text plus SD-list/download control lines."""
    global load1, load2, temp1, temp2, rpm, text_section
    global sd_list_active, sd_list_completed, sd_list_error, sd_list_count
    global logging_active, active_log_filename, log_record_count
    global download_active, download_expected_size, download_received_size
    global download_filename, download_last_status, download_stream_mode
    global download_end_crc16

    s = line.strip()
    if not s:
        return

    # Keep a small diagnostic history for the status dialog.
    with download_lock:
        teensy_text_log.append(s)

    upper = s.upper()

    # ---------------- SD LIST ----------------
    if upper == "SD_LIST_BEGIN":
        with sd_list_lock:
            sd_list_files.clear()
            sd_list_active = True
            sd_list_completed = False
            sd_list_error = ""
            sd_list_count = 0
        return

    if upper.startswith("FILE ") and sd_list_active:
        parts = s.split()
        if len(parts) >= 3:
            filename = parts[1]
            try:
                size = int(parts[2])
            except ValueError:
                size = 0
            with sd_list_lock:
                sd_list_files.append((filename, size))
        return

    if upper.startswith("SD_LIST_COUNT "):
        try:
            sd_list_count = int(s.split()[-1])
        except ValueError:
            pass
        return

    if upper == "SD_LIST_END":
        with sd_list_lock:
            sd_list_active = False
            sd_list_completed = True
        return

    if upper.startswith("ERROR:") and sd_list_active:
        with sd_list_lock:
            sd_list_error = s
            sd_list_active = False
            sd_list_completed = False
        return

    # ---------------- SD DOWNLOAD CONTROL LINES ----------------
    if upper.startswith("SD_DOWNLOAD_BEGIN "):
        parts = s.split(maxsplit=2)
        if len(parts) >= 3:
            filename = parts[1]
            try:
                total_size = int(parts[2])
            except ValueError:
                total_size = 0
            with download_lock:
                download_active = True
                download_expected_size = total_size
                download_received_size = 0
                download_filename = filename
                download_completed = False
                download_error = ""
                download_stream_mode = True
                download_crc16 = 0xFFFF
                download_end_crc16 = None
                download_last_status = f"Downloading {filename}..."
        return

    if upper.startswith("SD_DOWNLOAD_END "):
        parts = s.split()
        with download_lock:
            try:
                end_name = parts[1] if len(parts) > 1 else download_filename
                end_size = int(parts[2]) if len(parts) > 2 else download_received_size
                end_crc = int(parts[3], 16) if len(parts) > 3 else None
            except (ValueError, IndexError):
                end_name = download_filename
                end_size = download_received_size
                end_crc = None

            download_end_crc16 = end_crc
            download_stream_mode = False

            if download_file_handle is not None:
                try:
                    download_file_handle.flush()
                    download_file_handle.close()
                except Exception:
                    pass
                download_file_handle = None

            size_ok = (download_received_size == download_expected_size == end_size)
            crc_ok = (end_crc is None or end_crc == download_crc16)
            if size_ok and crc_ok:
                download_active = False
                download_completed = True
                download_last_status = (
                    f"Download complete: {download_received_size:,} bytes"
                )
            else:
                download_active = False
                download_completed = False
                if not size_ok:
                    download_error = (
                        f"Incomplete download: {download_received_size:,} / "
                        f"{download_expected_size:,} bytes"
                    )
                else:
                    download_error = (
                        f"Whole-file CRC mismatch: PC=0x{download_crc16:04X}, "
                        f"Teensy=0x{end_crc:04X}"
                    )
        return

    if upper.startswith("SD_DOWNLOAD_ERROR"):
        with download_lock:
            download_error = s
            download_active = False
            download_stream_mode = False
            if download_file_handle is not None:
                try:
                    download_file_handle.close()
                except Exception:
                    pass
                download_file_handle = None
        return

    # ---------------- LOGGING STATE ----------------
    if "SD LOGGING STARTED" in upper:
        logging_active = True
        return

    if "SD LOGGING STOPPED" in upper:
        logging_active = False
        return

    if upper.startswith("FILE:") and logging_active:
        active_log_filename = s.split(":", 1)[1].strip()
        return

    if upper.startswith("RECORDS:"):
        try:
            log_record_count = int(s.split(":", 1)[1].strip())
        except ValueError:
            pass
        return

    # ---------------- SENSOR TEXT ----------------
    if "LOAD CELL #1" in upper:
        text_section = "load1"
        return

    if "LOAD CELL #2" in upper:
        text_section = "load2"
        return

    if "MAX31856 #1" in upper:
        text_section = "temp1"
        return

    if "MAX31856 #2" in upper:
        text_section = "temp2"
        return

    if "WEIGHT:" in upper:
        try:
            value_text = s.split(":", 1)[1].replace("g", "").strip()
            value = float(value_text)
            with data_lock:
                if text_section == "load1":
                    load1 = value
                elif text_section == "load2":
                    load2 = value
        except ValueError:
            pass
        return

    if "THERMOCOUPLE:" in upper:
        try:
            value_text = (s.split(":", 1)[1]
                          .replace("°C", "")
                          .replace("C", "")
                          .strip())
            value = float(value_text)
            with data_lock:
                if text_section == "temp1":
                    temp1 = value
                elif text_section == "temp2":
                    temp2 = value
        except ValueError:
            pass
        return


# ============================================================
# TEENSY SD DOWNLOAD FRAME PARSER
# ============================================================

def handle_sd_download_frame(frame):
    """Parse D5 A5 01 OFFSET LENGTH DATA CRC16 frames from the Teensy."""
    global download_active, download_file_handle
    global download_received_size, download_error, download_last_status
    global download_crc16

    if len(frame) < SD_DOWNLOAD_HEADER_SIZE + SD_DOWNLOAD_CRC_SIZE:
        return False
    if frame[:2] != SD_DOWNLOAD_SYNC:
        return False

    frame_type = frame[2]
    offset = int.from_bytes(frame[3:7], "little")
    payload_len = int.from_bytes(frame[7:9], "little")
    expected_len = SD_DOWNLOAD_HEADER_SIZE + payload_len + SD_DOWNLOAD_CRC_SIZE
    if len(frame) != expected_len:
        return False
    if frame_type != SD_DOWNLOAD_TYPE_DATA:
        return False

    payload = frame[9:9 + payload_len]
    received_crc = int.from_bytes(frame[-2:], "little")
    calculated_crc = crc16_ccitt(frame[2:-2])
    if received_crc != calculated_crc:
        with download_lock:
            download_error = f"SD transfer CRC error at offset {offset:,}."
            download_active = False
            if download_file_handle is not None:
                try:
                    download_file_handle.close()
                except Exception:
                    pass
                download_file_handle = None
        return False

    with download_lock:
        if not download_active or download_file_handle is None:
            download_error = "Unexpected SD data frame."
            return False
        if offset != download_received_size:
            download_error = (
                f"SD transfer offset error: got {offset}, "
                f"expected {download_received_size}."
            )
            return False
        try:
            download_file_handle.write(payload)
            download_received_size += payload_len
            download_crc16 = crc16_update(download_crc16, payload)
            if download_expected_size:
                pct = 100.0 * download_received_size / download_expected_size
                download_last_status = (
                    f"Downloading {download_filename}: "
                    f"{download_received_size:,}/{download_expected_size:,} bytes "
                    f"({pct:.1f}%)"
                )
            else:
                download_last_status = f"Downloading: {download_received_size:,} bytes"
        except Exception as exc:
            download_error = f"PC file write error: {exc}"
            return False
    return True


# ============================================================
# SERIAL BINARY STREAM PARSER
# ============================================================

def serial_reader():
    """Continuous USB receiver for Teensy telemetry + SD transfer protocol."""
    global serial_connected, serial_command_serial, running
    global total_samples, good_packets, bad_packets
    global last_packet_arrival_wall_time, latest_sample_time, last_packet_sample_count
    global stream_start_wall_time

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0)
        ser.reset_input_buffer()
        serial_connected = True
        with serial_command_lock:
            serial_command_serial = ser
        print("----------------------------------------")
        print("TEENSY DAQ CONNECTED")
        print("Port:", SERIAL_PORT)
        print("Baud:", BAUD_RATE)
        print("----------------------------------------")
    except Exception as e:
        serial_connected = False
        print("Could not open serial port:", e)
        return

    rx = bytearray()
    text_bytes = bytearray()
    sample_index = 0

    def process_complete_text_lines():
        nonlocal text_bytes
        while b"\n" in text_bytes:
            line, _, remainder = text_bytes.partition(b"\n")
            text_bytes = bytearray(remainder)
            try:
                process_text_line(line.decode("utf-8", errors="ignore"))
            except Exception:
                pass

    while running:
        try:
            waiting = ser.in_waiting
            if waiting:
                chunk = ser.read(waiting)
                if chunk:
                    rx.extend(chunk)
            else:
                time.sleep(0.0005)

            while rx:
                # During a binary SD transfer, frames are back-to-back, followed
                # by an ASCII SD_DOWNLOAD_END line.
                if download_stream_mode:
                    if rx.startswith(SD_DOWNLOAD_SYNC):
                        if len(rx) < SD_DOWNLOAD_HEADER_SIZE:
                            break
                        payload_len = int.from_bytes(rx[7:9], "little")
                        frame_size = SD_DOWNLOAD_HEADER_SIZE + payload_len + SD_DOWNLOAD_CRC_SIZE
                        if len(rx) < frame_size:
                            break
                        frame = bytes(rx[:frame_size])
                        del rx[:frame_size]
                        handle_sd_download_frame(frame)
                        continue

                    # End/error lines after the final binary frame.
                    if rx.startswith(b"SD_DOWNLOAD_END") or rx.startswith(b"SD_DOWNLOAD_ERROR"):
                        newline = rx.find(b"\n")
                        if newline < 0:
                            break
                        line = bytes(rx[:newline])
                        del rx[:newline + 1]
                        process_text_line(line.decode("utf-8", errors="ignore"))
                        continue

                    # Partial ASCII line may have arrived after the final frame.
                    if rx[:1] in b"SDE" and b"\n" not in rx and not rx.startswith(SD_DOWNLOAD_SYNC):
                        break

                    # Resynchronise on the next D5 A5 frame.
                    sync = rx.find(SD_DOWNLOAD_SYNC)
                    if sync > 0:
                        del rx[:sync]
                        continue
                    if sync < 0:
                        # Preserve a possible partial command prefix.
                        keep = max(0, min(len(rx), len(b"SD_DOWNLOAD_END") - 1))
                        if keep:
                            text_bytes.extend(rx[:-keep])
                            del rx[:-keep]
                        break

                # Normal mode: find whichever comes first, telemetry AA55 or
                # the SD transfer binary sync. Text preceding it is parsed as
                # human-readable telemetry/control lines.
                tele_pos = rx.find(bytes([SYNC1, SYNC2]))
                sd_pos = rx.find(SD_DOWNLOAD_SYNC)

                candidates = [p for p in (tele_pos, sd_pos) if p >= 0]
                sync_pos = min(candidates) if candidates else -1

                if sync_pos == -1:
                    text_bytes.extend(rx)
                    rx.clear()
                    process_complete_text_lines()
                    break

                if sync_pos > 0:
                    text_bytes.extend(rx[:sync_pos])
                    del rx[:sync_pos]
                    process_complete_text_lines()
                    continue

                # SD binary frame should only occur after SD_DOWNLOAD_BEGIN.
                if rx.startswith(SD_DOWNLOAD_SYNC):
                    if len(rx) < SD_DOWNLOAD_HEADER_SIZE:
                        break
                    payload_len = int.from_bytes(rx[7:9], "little")
                    frame_size = SD_DOWNLOAD_HEADER_SIZE + payload_len + SD_DOWNLOAD_CRC_SIZE
                    if len(rx) < frame_size:
                        break
                    frame = bytes(rx[:frame_size])
                    del rx[:frame_size]
                    handle_sd_download_frame(frame)
                    continue

                # AA55 acceleration packet.
                if len(rx) < HEADER_SIZE:
                    break
                packet_type = rx[2]
                count = rx[3] | (rx[4] << 8)
                if packet_type != PACKET_TYPE_ACCEL or count <= 0 or count > 500:
                    del rx[0]
                    continue

                payload_size = count * 6
                packet_size = HEADER_SIZE + payload_size + CRC_SIZE
                if len(rx) < packet_size:
                    break

                packet = bytes(rx[:packet_size])
                del rx[:packet_size]
                payload = packet[HEADER_SIZE:HEADER_SIZE + payload_size]
                received_crc = int.from_bytes(packet[-2:], "little")
                calculated_crc = crc16_ccitt(packet[2:-2])

                if received_crc != calculated_crc:
                    bad_packets += 1
                    continue

                good_packets += 1
                packet_arrival_wall_time = time.time()
                if stream_start_wall_time is None:
                    stream_start_wall_time = packet_arrival_wall_time - (count - 1) / ACC_SAMPLE_RATE_HZ

                with data_lock:
                    for i in range(count):
                        p = i * 6
                        raw_x = payload[p] | (payload[p + 1] << 8)
                        raw_y = payload[p + 2] | (payload[p + 3] << 8)
                        raw_z = payload[p + 4] | (payload[p + 5] << 8)
                        gx = adc_to_g(raw_x, ADXL_OFFSET_X_V, ADXL_SENS_X_V_PER_G)
                        gy = adc_to_g(raw_y, ADXL_OFFSET_Y_V, ADXL_SENS_Y_V_PER_G)
                        gz = adc_to_g(raw_z, ADXL_OFFSET_Z_V, ADXL_SENS_Z_V_PER_G)
                        t = stream_start_wall_time + sample_index / ACC_SAMPLE_RATE_HZ
                        time_data.append(t)
                        vib_x.append(gx)
                        vib_y.append(gy)
                        vib_z.append(gz)
                        sample_index += 1
                        total_samples += 1

                latest_sample_time = stream_start_wall_time + (sample_index - 1) / ACC_SAMPLE_RATE_HZ
                last_packet_arrival_wall_time = packet_arrival_wall_time
                last_packet_sample_count = count

            # Parse any complete text that remains outside binary packets.
            process_complete_text_lines()

        except serial.SerialException as e:
            print("Serial error:", e)
            break
        except Exception as e:
            print("Reader error:", e)
            break

    try:
        ser.close()
    except Exception:
        pass
    with serial_command_lock:
        if serial_command_serial is ser:
            serial_command_serial = None
    serial_connected = False


# ============================================================
# SEND COMMAND TO TEENSY
# ============================================================

def send_teensy_command(command):
    """Send one line command to the Teensy over the telemetry serial port."""
    command = str(command).strip()

    if not command:
        return False, "Command is empty."

    if not command.endswith("\n"):
        command += "\n"

    with serial_command_lock:
        ser = serial_command_serial

    if ser is None or not ser.is_open:
        return False, "Serial port is not connected."

    try:
        with serial_command_lock:
            ser.write(command.encode("utf-8"))
            ser.flush()
        print("TX -> Teensy:", command.strip())
        return True, f"Sent: {command.strip()}"
    except Exception as e:
        return False, f"Send error: {e}"


def prepare_sd_download(target_path):
    """Open the PC destination before asking the Teensy to transmit."""
    global download_file_handle, download_active, download_target_path
    global download_expected_size, download_received_size, download_error
    global download_completed, download_filename, download_last_status
    global download_stream_mode, download_crc16, download_end_crc16

    with download_lock:
        if download_active or download_file_handle is not None:
            return False, "A download is already in progress."
        try:
            download_file_handle = open(target_path, "wb")
        except Exception as exc:
            return False, f"Cannot create PC file: {exc}"
        download_target_path = target_path
        download_expected_size = 0
        download_received_size = 0
        download_error = ""
        download_completed = False
        download_filename = ""
        download_stream_mode = False
        download_crc16 = 0xFFFF
        download_end_crc16 = None
        download_last_status = "Waiting for Teensy..."
    return True, "PC destination ready."


def cancel_sd_download_file():
    global download_file_handle, download_active, download_stream_mode
    with download_lock:
        download_active = False
        download_stream_mode = False
        if download_file_handle is not None:
            try:
                download_file_handle.close()
            except Exception:
                pass
            download_file_handle = None


def request_sd_list():
    global sd_list_active, sd_list_completed, sd_list_error
    with sd_list_lock:
        sd_list_files.clear()
        sd_list_active = True
        sd_list_completed = False
        sd_list_error = ""
    ok, msg = send_teensy_command("SD_LIST")
    if not ok:
        with sd_list_lock:
            sd_list_active = False
            sd_list_error = msg
        return False, msg
    return True, "SD file list requested."


def get_sd_list_state():
    with sd_list_lock:
        return list(sd_list_files), sd_list_active, sd_list_completed, sd_list_error


def request_sd_download(filename, target_path):
    filename = str(filename).strip()
    if not filename or any(c in filename for c in "\\/\r\n"):
        return False, "Invalid SD filename."
    if not filename.lower().endswith(".bin"):
        filename += ".bin"

    ok, message = prepare_sd_download(target_path)
    if not ok:
        return False, message

    ok, message = send_teensy_command(f"SD_DOWNLOAD {filename}")
    if not ok:
        cancel_sd_download_file()
        return False, message
    return True, f"Download requested: {filename}"

def start_teensy_log(filename):
    filename = str(filename).strip()

    if not filename:
        return False, "Enter a log filename."

    # Prevent accidental command injection through the filename field.
    if any(c in filename for c in "\r\n"):
        return False, "Filename cannot contain Enter/new-line characters."

    return send_teensy_command(f"LOGSTART {filename}")


def stop_teensy_log():
    return send_teensy_command("LOGSTOP")


# REAL-TIME CLOCK AXIS
# Displays wall-clock time (HH:MM:SS) while the internal x
# coordinate remains Unix time in seconds.
# ============================================================

class FixedTimeAxis(pg.AxisItem):
    """
    Fixed screen-space time axis.

    The plot X coordinates are relative to the moving playhead:
        -PLOT_WINDOW_SECONDS ... 0

    Therefore the grid stays fixed on the screen instead of drifting
    with the Unix/wall-clock X coordinate.

    Labels are generated from the current wall-clock time represented
    by the playhead.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reference_wall_time = time.time()

    def set_reference_time(self, wall_time):
        self.reference_wall_time = wall_time

    def tickValues(self, minVal, maxVal, size):
        # Fixed tick positions in the plot coordinate system.
        # Major grid every 1 s, minor grid every 0.2 s.
        major_start = int(np.ceil(minVal))
        major_end = int(np.floor(maxVal))

        major = list(
            np.arange(major_start, major_end + 1, 1.0)
        )

        minor_start = int(np.ceil(minVal / 0.2))
        minor_end = int(np.floor(maxVal / 0.2))

        minor = [
            i * 0.2
            for i in range(minor_start, minor_end + 1)
            if abs((i * 0.2) - round(i)) > 1e-9
        ]

        return [
            (1.0, major),
            (0.2, minor)
        ]

    def tickStrings(self, values, scale, spacing):
        result = []

        for value in values:
            try:
                wall_time = self.reference_wall_time + float(value)
                result.append(
                    time.strftime(
                        "%H:%M:%S",
                        time.localtime(wall_time)
                    )
                )
            except Exception:
                result.append("")

        return result


# ============================================================
# DISPLAY-ONLY SMOOTHING + BUFFERED DISPLAY
# ============================================================

def smooth_for_display(values, window=5):
    """
    Visual-only moving average.
    Raw 1 kHz DAQ samples are never modified.
    """
    n = len(values)

    if not DISPLAY_SMOOTHING or window <= 1 or n < window:
        return np.asarray(values, dtype=np.float64)

    window = max(3, int(window))
    if window % 2 == 0:
        window += 1

    arr = np.asarray(values, dtype=np.float64)

    half = window // 2

    padded = np.pad(
        arr,
        (half, half),
        mode="edge"
    )

    kernel = np.ones(
        window,
        dtype=np.float64
    ) / window

    return np.convolve(
        padded,
        kernel,
        mode="valid"
    )



class ScalarKalman:
    """
    Very small real-time 1D Kalman estimator.

    This is a display-only filter. It does not alter raw data stored
    in the deques and does not alter the Teensy SD binary log.

    q = process noise: higher -> follows vibration more aggressively.
    r = measurement noise: higher -> smoother but slower response.
    """
    def __init__(self, q=0.02, r=0.10, initial_variance=1.0):
        self.q = float(q)
        self.r = float(r)
        self.x = 0.0
        self.p = float(initial_variance)
        self.initialized = False

    def reset(self):
        self.x = 0.0
        self.p = 1.0
        self.initialized = False

    def filter_array(self, values):
        arr = np.asarray(values, dtype=np.float64)

        if arr.size == 0:
            return arr

        out = np.empty_like(arr)

        if not self.initialized:
            self.x = float(arr[0])
            self.initialized = True

        for i, measurement in enumerate(arr):
            # Predict.
            self.p += self.q

            # Update.
            k = self.p / (self.p + self.r)
            self.x += k * (float(measurement) - self.x)
            self.p = (1.0 - k) * self.p

            out[i] = self.x

        return out


kalman_x = ScalarKalman(KALMAN_PROCESS_NOISE,
                        KALMAN_MEASUREMENT_NOISE)
kalman_y = ScalarKalman(KALMAN_PROCESS_NOISE,
                        KALMAN_MEASUREMENT_NOISE)
kalman_z = ScalarKalman(KALMAN_PROCESS_NOISE,
                        KALMAN_MEASUREMENT_NOISE)


def make_buffered_display(t, x, y, z, playhead_time):
    """
    Build the display frame around a continuously moving playhead.

    Acquisition:
        1000 samples/s

    Playback:
        starts after 1 s of data has been acquired
        and follows the acquisition 0.5 s behind.

    The playhead itself is advanced by the 60 Hz GUI timer, not
    by packet arrival. Therefore the plot scrolls smoothly.
    """

    if len(t) < 10 or playhead_time is None:
        return None

    t = np.asarray(t, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)

    valid = (
        np.isfinite(t)
        & np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
    )

    t = t[valid]
    x = x[valid]
    y = y[valid]
    z = z[valid]

    if len(t) < 10:
        return None

    display_start = playhead_time - PLOT_WINDOW_SECONDS
    display_end = playhead_time

    mask = (t >= display_start) & (t <= display_end)

    display_t = t[mask]
    display_x = x[mask]
    display_y = y[mask]
    display_z = z[mask]

    if len(display_t) < 10:
        return None

    # Visual-only smoothing. X timestamps remain untouched and
    # remain exactly 1 ms apart.
    if DISPLAY_SMOOTHING:
        display_x = smooth_for_display(
            display_x,
            SMOOTHING_WINDOW
        )
        display_y = smooth_for_display(
            display_y,
            SMOOTHING_WINDOW
        )
        display_z = smooth_for_display(
            display_z,
            SMOOTHING_WINDOW
        )

    # Optional zero-lookahead Kalman filtering.
    # It is applied only to the displayed arrays; raw samples are
    # never changed. This avoids adding a fixed moving-average delay.
    if KALMAN_FILTER_ENABLED:
        display_x = kalman_x.filter_array(display_x)
        display_y = kalman_y.filter_array(display_y)
        display_z = kalman_z.filter_array(display_z)

    # IMPORTANT:
    # Convert wall-clock timestamps to a fixed coordinate system.
    # The right edge is always 0 and the left edge is always
    # -PLOT_WINDOW_SECONDS. This makes the grid stationary.
    relative_t = display_t - playhead_time

    return (
        relative_t,
        display_x,
        display_y,
        display_z
    )


# ============================================================
# GUI
# ============================================================

class LegacyDAQMonitor(QtWidgets.QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "Teensy 4.1 Engine DAQ Monitor"
        )

        self.resize(1500, 900)

        self.setup_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self.update_gui)

        self.timer.start(
            max(1, int(round(1000.0 / GUI_UPDATE_HZ)))
        )

    # --------------------------------------------------------
    # UI
    # --------------------------------------------------------

    def setup_ui(self):

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main = QtWidgets.QVBoxLayout(central)

        # ----------------------------------------------------
        # Sensor cards
        # ----------------------------------------------------

        grid = QtWidgets.QGridLayout()

        self.rpm_label = self.sensor_card(
            "RPM",
            "N/A"
        )

        self.load1_label = self.sensor_card(
            "LOAD CELL 1",
            "0.00 g"
        )

        self.load2_label = self.sensor_card(
            "LOAD CELL 2",
            "0.00 g"
        )

        self.temp1_label = self.sensor_card(
            "TEMP 1 / EGT",
            "N/A"
        )

        self.temp2_label = self.sensor_card(
            "TEMP 2 / CHT",
            "N/A"
        )

        self.connection_label = self.sensor_card(
            "SERIAL",
            "DISCONNECTED"
        )

        grid.addWidget(self.rpm_label, 0, 0)
        grid.addWidget(self.load1_label, 0, 1)
        grid.addWidget(self.load2_label, 0, 2)
        grid.addWidget(self.temp1_label, 0, 3)
        grid.addWidget(self.temp2_label, 0, 4)
        grid.addWidget(self.connection_label, 0, 5)

        main.addLayout(grid)

        # ----------------------------------------------------
        # Vibration plot
        # ----------------------------------------------------

        self.time_axis = FixedTimeAxis(orientation="bottom")

        self.plot = pg.PlotWidget(
            axisItems={"bottom": self.time_axis}
        )

        self.plot.setBackground("k")

        self.plot.setTitle(
            "ADXL335 Real-Time Vibration"
        )

        self.plot.setLabel(
            "left",
            "Acceleration",
            units="g"
        )

        self.plot.setLabel(
            "bottom",
            "Time",
            units="wall-clock"
        )

        self.plot.showGrid(
            x=True,
            y=True,
            alpha=0.3
        )

        self.plot.addLegend()

        self.curve_x = self.plot.plot(
            pen=pg.mkPen(color="#ff3b30", width=2.0),
            name="X"
        )

        self.curve_y = self.plot.plot(
            pen=pg.mkPen(color="#34c759", width=2.4),
            name="Y"
        )

        self.curve_z = self.plot.plot(
            pen=pg.mkPen(color="#0a84ff", width=2.0),
            name="Z"
        )

        # Display optimization only. Raw samples are still retained
        # in Python memory at the full 2 kHz rate.
        self.curve_x.setDownsampling(
            auto=True,
            method="peak"
        )
        self.curve_y.setDownsampling(
            auto=True,
            method="peak"
        )
        self.curve_z.setDownsampling(
            auto=True,
            method="peak"
        )

        self.curve_x.setClipToView(True)
        self.curve_y.setClipToView(True)
        self.curve_z.setClipToView(True)

        main.addWidget(self.plot)

        # ----------------------------------------------------
        # Status
        # ----------------------------------------------------

        self.statusBar().showMessage(
            "Starting..."
        )

        # ----------------------------------------------------
        # Teensy SD LOG CONTROL
        # ----------------------------------------------------

        log_control = QtWidgets.QGroupBox("TEENSY SD LOG CONTROL")
        log_layout = QtWidgets.QHBoxLayout(log_control)

        log_layout.addWidget(
            QtWidgets.QLabel("Filename:")
        )

        self.log_filename_input = QtWidgets.QLineEdit()
        self.log_filename_input.setText(LOG_FILENAME_DEFAULT)
        self.log_filename_input.setPlaceholderText("e.g. test01")
        self.log_filename_input.setMinimumWidth(220)
        self.log_filename_input.returnPressed.connect(
            self.start_log_from_gui
        )
        log_layout.addWidget(self.log_filename_input)

        self.start_log_button = QtWidgets.QPushButton("START LOG")
        self.start_log_button.clicked.connect(
            self.start_log_from_gui
        )
        log_layout.addWidget(self.start_log_button)

        self.stop_log_button = QtWidgets.QPushButton("STOP LOG")
        self.stop_log_button.clicked.connect(
            self.stop_log_from_gui
        )
        log_layout.addWidget(self.stop_log_button)

        self.command_input = QtWidgets.QLineEdit()
        self.command_input.setPlaceholderText(
            "Optional Teensy command, e.g. STATUS"
        )
        self.command_input.returnPressed.connect(
            self.send_custom_command_from_gui
        )
        log_layout.addWidget(self.command_input)

        self.send_command_button = QtWidgets.QPushButton("SEND COMMAND")
        self.send_command_button.clicked.connect(
            self.send_custom_command_from_gui
        )
        log_layout.addWidget(self.send_command_button)

        self.download_filename_input = QtWidgets.QLineEdit()
        self.download_filename_input.setText(SD_DOWNLOAD_DEFAULT)
        self.download_filename_input.setPlaceholderText("SD file, e.g. test01.bin")
        self.download_filename_input.setMinimumWidth(220)
        self.download_filename_input.returnPressed.connect(
            self.download_sd_file_from_gui
        )
        log_layout.addWidget(self.download_filename_input)

        self.download_sd_button = QtWidgets.QPushButton("DOWNLOAD SD FILE")
        self.download_sd_button.clicked.connect(
            self.download_sd_file_from_gui
        )
        log_layout.addWidget(self.download_sd_button)

        main.addWidget(log_control)

        self.log_status_label = QtWidgets.QLabel(
            "Log control: Ready"
        )
        main.addWidget(self.log_status_label)
        self.download_sd_button.setEnabled(True)

    # --------------------------------------------------------
    # Sensor card
    # --------------------------------------------------------

    def sensor_card(self, title, value):

        label = QtWidgets.QLabel(
            f"{title}\n{value}"
        )

        label.setAlignment(
            QtCore.Qt.AlignCenter
        )

        label.setMinimumHeight(85)

        label.setStyleSheet("""
            QLabel {
                background-color: #202020;
                border: 1px solid #555;
                border-radius: 8px;
                color: white;
                font-size: 18px;
                font-weight: bold;
                padding: 8px;
            }
        """)

        return label

    # --------------------------------------------------------
    # Update GUI
    # --------------------------------------------------------


    # --------------------------------------------------------
    # Teensy command buttons
    # --------------------------------------------------------

    def start_log_from_gui(self):
        filename = self.log_filename_input.text().strip()

        if not filename:
            self.log_status_label.setText(
                "Log control: Enter a filename."
            )
            return

        ok, message = start_teensy_log(filename)

        self.log_status_label.setText(
            "Log control: " + message
        )

        if ok:
            self.start_log_button.setEnabled(False)
            self.stop_log_button.setEnabled(True)

    def stop_log_from_gui(self):
        ok, message = stop_teensy_log()

        self.log_status_label.setText(
            "Log control: " + message
        )

        if ok:
            self.start_log_button.setEnabled(True)
            self.stop_log_button.setEnabled(False)

    def download_sd_file_from_gui(self):
        filename = self.download_filename_input.text().strip()
        if not filename:
            self.log_status_label.setText("SD download: Enter an SD filename.")
            return
        if any(c in filename for c in "\\/\r\n"):
            self.log_status_label.setText("SD download: Enter only a filename, not a path.")
            return

        target_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Teensy SD File",
            filename,
            "Binary files (*.bin);;All files (*)"
        )

        if not target_path:
            self.log_status_label.setText("SD download: Cancelled.")
            return

        # Prevent accidental overwrite unless the user explicitly confirms it.
        ok, message = request_sd_download(filename, target_path)
        self.log_status_label.setText("SD download: " + message)
        if ok:
            self.download_sd_button.setEnabled(False)

    def send_custom_command_from_gui(self):
        command = self.command_input.text().strip()

        if not command:
            self.log_status_label.setText(
                "Command: Enter a command."
            )
            return

        ok, message = send_teensy_command(command)

        self.log_status_label.setText(
            "Command: " + message
        )

        if ok:
            self.command_input.clear()

    def update_gui(self):

        with data_lock:

            x = list(vib_x)
            y = list(vib_y)
            z = list(vib_z)
            t = list(time_data)

            current_load1 = load1
            current_load2 = load2
            current_temp1 = temp1
            current_temp2 = temp2

            current_latest_sample_time = latest_sample_time
            current_packet_arrival_time = last_packet_arrival_wall_time
            current_packet_sample_count = last_packet_sample_count

        with download_lock:
            current_download_active = download_active
            current_download_received = download_received_size
            current_download_expected = download_expected_size
            current_download_error = download_error
            current_download_completed = download_completed
            current_download_status = download_last_status

        if current_download_error:
            self.log_status_label.setText("SD download: " + current_download_error)
            self.download_sd_button.setEnabled(False)
        elif current_download_completed:
            self.log_status_label.setText("SD download: " + current_download_status)
            self.download_sd_button.setEnabled(True)
        elif current_download_active:
            self.log_status_label.setText("SD download: " + current_download_status)
            self.download_sd_button.setEnabled(False)

        # ----------------------------------------------------
        # Text values
        # ----------------------------------------------------

        self.load1_label.setText(
            f"LOAD CELL 1\n"
            f"{current_load1:.2f} {LOAD_UNIT}"
        )

        self.load2_label.setText(
            f"LOAD CELL 2\n"
            f"{current_load2:.2f} {LOAD_UNIT}"
        )

        if current_temp1 == current_temp1:
            self.temp1_label.setText(
                f"TEMP 1 / EGT\n"
                f"{current_temp1:.2f} {TEMP_UNIT}"
            )

        if current_temp2 == current_temp2:
            self.temp2_label.setText(
                f"TEMP 2 / CHT\n"
                f"{current_temp2:.2f} {TEMP_UNIT}"
            )

        if serial_connected:
            self.connection_label.setText(
                "SERIAL\nCONNECTED"
            )
        else:
            self.connection_label.setText(
                "SERIAL\nDISCONNECTED"
            )

        # RPM is NOT transmitted by the current Teensy code.
        self.rpm_label.setText(
            "RPM\nN/A"
        )

        # ----------------------------------------------------
        # SMOOTH 60 Hz PLAYBACK
        # ----------------------------------------------------
        #
        # We intentionally do NOT set the X range to the latest
        # packet timestamp.
        #
        # Instead:
        #   1. acquire 1.0 s of data
        #   2. start the display
        #   3. keep it 0.5 s behind acquisition
        #   4. advance the playhead at the GUI's 60 Hz rate
        #
        # This makes the plot slide continuously.
        # ----------------------------------------------------

        global display_playhead_time
        global display_started
        global last_gui_wall_time

        if len(t) > 2:

            n = min(len(t), len(x), len(y), len(z))
            t = t[-n:]
            x = x[-n:]
            y = y[-n:]
            z = z[-n:]

            # ----------------------------------------------------
            # ZERO-DELAY LIVE PLAYHEAD
            # ----------------------------------------------------
            if current_latest_sample_time is not None:

                packet_period = (
                    max(1, current_packet_sample_count)
                    / ACC_SAMPLE_RATE_HZ
                )

                if current_packet_arrival_time is not None:
                    wall_elapsed = (
                        time.time() - current_packet_arrival_time
                    )

                    # Predict only within the current packet interval.
                    wall_elapsed = min(
                        max(wall_elapsed, 0.0),
                        packet_period
                    )

                    live_time = (
                        current_latest_sample_time
                        + wall_elapsed
                    )
                else:
                    live_time = current_latest_sample_time

                if display_playhead_time is None:
                    display_playhead_time = live_time
                else:
                    # Follow the live edge immediately. Never introduce
                    # a deliberate delay or playback buffer.
                    display_playhead_time = max(
                        display_playhead_time,
                        live_time
                    )

                display_started = True
                last_gui_wall_time = time.perf_counter()

                display = make_buffered_display(
                    t,
                    x,
                    y,
                    z,
                    display_playhead_time
                )

                if display is not None:

                    (
                        display_t,
                        display_x,
                        display_y,
                        display_z
                    ) = display

                    self.curve_x.setData(
                        display_t,
                        display_x,
                        skipFiniteCheck=True
                    )

                    self.curve_y.setData(
                        display_t,
                        display_y,
                        skipFiniteCheck=True
                    )

                    self.curve_z.setData(
                        display_t,
                        display_z,
                        skipFiniteCheck=True
                    )

                    self.plot.setXRange(
                        -PLOT_WINDOW_SECONDS,
                        0.0,
                        padding=0
                    )

                    self.time_axis.set_reference_time(
                        display_playhead_time
                    )

        self.statusBar().showMessage(
            f"Packets OK: {good_packets} | "
            f"CRC errors: {bad_packets} | "
            f"Samples: {total_samples} | "
            f"ADXL: {ACC_SAMPLE_RATE_HZ} Hz | "
            f"Packets: 100/s | "
            f"Samples/packet: 20 | "
            f"GUI: {GUI_UPDATE_HZ} Hz | "
            f"Kalman: {'ON' if KALMAN_FILTER_ENABLED else 'OFF'} | "
            f"REAL-TIME / 0 ms artificial delay"
        )

    # --------------------------------------------------------
    # Close
    # --------------------------------------------------------

    def closeEvent(self, event):

        global running

        running = False

        event.accept()



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


# ============================================================
# udp_connection.py
# ENGINE TEST BENCH DAQ
#
# PC GUI <---- UDP / mDNS ----> ESP32 DAQ
#
# ESP32:
#   mDNS name : engine-daq.local
#   service   : _engine-daq._udp.local.
#   UDP port  : 4210
#
# PC -> ESP32 commands:
#   HELLO
#   PING
#   READY
#   RUN
#   STOP
#   SET_THROTTLE
#   UPLOAD_CONFIG
#
# ESP32 -> PC:
#   hello
#   state
#   telemetry
#   ack
#   error
#
# Telemetry target:
#   60 Hz
# ============================================================

import json
import socket
import threading
import time

from PyQt5.QtCore import QObject, pyqtSignal

try:
    from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
except ImportError:
    Zeroconf = None
    ServiceBrowser = None
    ServiceListener = object


# ============================================================
# mDNS LISTENER
# ============================================================

class ESP32DiscoveryListener(ServiceListener):

    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def add_service(self, zeroconf, service_type, name):
        try:
            info = zeroconf.get_service_info(
                service_type,
                name
            )

            if info:
                addresses = info.parsed_addresses()

                if addresses:
                    self.callback(
                        addresses[0],
                        info.port
                    )

        except Exception as e:
            print("mDNS discovery error:", e)

    def update_service(self, zeroconf, service_type, name):
        self.add_service(
            zeroconf,
            service_type,
            name
        )

    def remove_service(self, zeroconf, service_type, name):
        pass


# ============================================================
# ESP32 UDP CONNECTION
# ============================================================

class ESP32UDPConnection(QObject):

    # --------------------------------------------------------
    # SIGNALS
    # --------------------------------------------------------

    connectionChanged = pyqtSignal(bool)

    stateChanged = pyqtSignal(str)

    telemetryReceived = pyqtSignal(dict)

    messageReceived = pyqtSignal(dict)

    errorOccurred = pyqtSignal(str)

    deviceFound = pyqtSignal(str, int)

    # --------------------------------------------------------
    # INIT
    # --------------------------------------------------------

    def __init__(self, parent=None):
        super().__init__(parent)

        # IMPORTANT:
        # These must always exist.
        # Prevents:
        # AttributeError: Dashboard has no attribute connected
        self.connected = False

        self.socket = None

        self.esp_ip = None
        self.esp_port = 4210

        self.running = False

        self.discovery = None
        self.browser = None

        self.receive_thread = None

        self.last_packet_time = 0.0

        self.device_name = "engine-daq.local"

        self.state = "DISCONNECTED"

        self.sequence_number = 0

        self.lock = threading.Lock()

    # ========================================================
    # mDNS DISCOVERY
    # ========================================================

    def discover(self):

        if Zeroconf is None:
            self.errorOccurred.emit(
                "zeroconf is not installed. "
                "Install using: pip install zeroconf"
            )
            return

        try:

            if self.discovery is not None:
                return

            self.discovery = Zeroconf()

            listener = ESP32DiscoveryListener(
                self.on_device_found
            )

            self.browser = ServiceBrowser(
                self.discovery,
                "_engine-daq._udp.local.",
                listener
            )

        except Exception as e:

            self.errorOccurred.emit(
                f"mDNS discovery failed: {e}"
            )

    # ========================================================
    # DEVICE FOUND
    # ========================================================

    def on_device_found(self, ip, port):

        self.esp_ip = ip
        self.esp_port = int(port)

        self.deviceFound.emit(
            self.esp_ip,
            self.esp_port
        )

        print(
            f"[mDNS] ESP32 found at "
            f"{self.esp_ip}:{self.esp_port}"
        )

        self.open_socket()

        # Immediately handshake
        self.send({
            "command": "HELLO"
        })

    # ========================================================
    # MANUAL CONNECTION
    # ========================================================

    def connect_ip(self, ip, port=4210):

        self.esp_ip = ip
        self.esp_port = int(port)

        self.open_socket()

        return self.send({
            "command": "HELLO"
        })

    # ========================================================
    # UDP SOCKET
    # ========================================================

    def open_socket(self):

        if self.socket is not None:
            return True

        try:

            self.socket = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )

            # Reuse local UDP socket
            self.socket.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1
            )

            self.socket.settimeout(0.5)

            # Let Windows choose local port
            self.socket.bind(
                ("0.0.0.0", 0)
            )

            self.running = True

            self.receive_thread = threading.Thread(
                target=self.receive_loop,
                daemon=True
            )

            self.receive_thread.start()

            print("[UDP] Socket opened")

            return True

        except Exception as e:

            self.socket = None

            self.errorOccurred.emit(
                f"UDP socket error: {e}"
            )

            return False

    # ========================================================
    # SEND JSON
    # ========================================================

    def send(self, message):

        if self.socket is None:
            self.errorOccurred.emit(
                "UDP socket is not open"
            )
            return False

        if not self.esp_ip:
            self.errorOccurred.emit(
                "ESP32 IP address not known"
            )
            return False

        try:

            message = dict(message)

            self.sequence_number += 1

            message.setdefault(
                "seq",
                self.sequence_number
            )

            message.setdefault(
                "timestamp_ms",
                int(time.time() * 1000)
            )

            payload = json.dumps(
                message,
                separators=(",", ":")
            ).encode("utf-8")

            print(f"[DAQ] UDP TX: {payload.decode('utf-8')}")

            with self.lock:

                self.socket.sendto(
                    payload,
                    (
                        self.esp_ip,
                        self.esp_port
                    )
                )

            return True

        except Exception as e:

            self.errorOccurred.emit(
                f"UDP TX error: {e}"
            )

            return False

    # ========================================================
    # RECEIVE LOOP
    # ========================================================

    def receive_loop(self):

        while self.running:

            try:

                data, address = self.socket.recvfrom(
                    8192
                )

                self.last_packet_time = time.monotonic()

                try:

                    message = json.loads(
                        data.decode("utf-8")
                    )

                    print(f"[DAQ] UDP RX: {message}")

                except Exception as e:

                    self.errorOccurred.emit(
                        f"Invalid JSON received: {e}"
                    )

                    continue

                self.process_message(
                    message
                )

            except socket.timeout:
                continue

            except OSError:

                if self.running:
                    self.errorOccurred.emit(
                        "UDP socket closed"
                    )

                break

            except Exception as e:

                if self.running:

                    self.errorOccurred.emit(
                        f"UDP RX error: {e}"
                    )

    # ========================================================
    # PROCESS MESSAGE
    # ========================================================

    def process_message(self, message):

        self.messageReceived.emit(
            message
        )

        message_type = str(
            message.get("type", message.get("command", ""))
        ).lower()

        # ----------------------------------------------------
        # HELLO / PING
        # ----------------------------------------------------

        if message_type in ("hello", "ping", "ack"):

            self.set_connected(
                True
            )

            state = str(
                message.get("state", message.get("status", "READY"))
            ).upper()

            self.set_state(
                state
            )

            return

        # ----------------------------------------------------
        # STATE / STATUS
        # ----------------------------------------------------

        if message_type in ("state", "status", "ready"):

            self.set_connected(
                True
            )

            state = str(
                message.get("state", message.get("status", "READY"))
            ).upper()

            self.set_state(
                state
            )

            return

        # ----------------------------------------------------
        # START / STOP
        # ----------------------------------------------------

        if message_type in ("daq_start", "start"):
            self.set_connected(True)
            self.set_state("RUNNING")
            return

        if message_type in ("stop", "daq_stop"):
            self.set_connected(True)
            self.set_state("STOPPED")
            return

        # ----------------------------------------------------
        # TELEMETRY
        # ----------------------------------------------------

        if message_type in ("telemetry", "telem", "teensy_telemetry"):

            self.set_connected(
                True
            )

            self.telemetryReceived.emit(
                message
            )

            state = str(
                message.get("state", "RUNNING")
            ).upper()

            self.set_state(
                state
            )

            return

        # ----------------------------------------------------
        # ERROR
        # ----------------------------------------------------

        if message_type == "error":

            self.errorOccurred.emit(
                message.get(
                    "message",
                    message.get("error", "ESP32 error")
                )
            )

            return

    # ========================================================
    # CONNECTION STATE
    # ========================================================

    def set_connected(self, value):

        value = bool(value)

        if self.connected != value:

            self.connected = value

            self.connectionChanged.emit(
                value
            )

    # ========================================================
    # STATE
    # ========================================================

    def set_state(self, state):

        new_state = str(state).upper()
        if self.state == new_state:
            return

        self.state = new_state

        self.stateChanged.emit(
            self.state
        )

    # ========================================================
    # UPLOAD CONFIG
    # ========================================================

    def upload_config(self, config):

        return self.send({
            "command": "UPLOAD_CONFIG",
            "config": config
        })

    # ========================================================
    # READY
    # ========================================================

    def ready(self):

        return self.send({
            "command": "READY"
        })

    # ========================================================
    # RUN
    # ========================================================

    def run(self):
        return self.send({"command": "RUN"})

    # ========================================================
    # STOP
    # ========================================================

    def stop(self):
        return self.send({"command": "STOP"})

    # ========================================================
    # THROTTLE
    # ========================================================

    def set_throttle(self, value):

        value = max(
            0.0,
            min(
                100.0,
                float(value)
            )
        )

        return self.send({
            "command": "SET_THROTTLE",
            "value": value
        })

    # ========================================================
    # SEQUENCE PROFILE
    # ========================================================

    def upload_profile(self, profile):

        return self.send({
            "command": "UPLOAD_PROFILE",
            "profile": profile
        })

    # ========================================================
    # START PROFILE
    # ========================================================

    def start_profile(self):

        return self.send({
            "command": "START_PROFILE"
        })

    # ========================================================
    # STOP PROFILE
    # ========================================================

    def stop_profile(self):

        return self.send({
            "command": "STOP_PROFILE"
        })

    # ========================================================
    # PING
    # ========================================================

    def ping(self):

        return self.send({
            "command": "PING"
        })

    # ========================================================
    # DISCONNECT
    # ========================================================

    def disconnect(self):

        self.running = False

        self.set_connected(
            False
        )

        self.set_state(
            "DISCONNECTED"
        )

        if self.socket:

            try:
                self.socket.close()

            except Exception:
                pass

            self.socket = None

        if self.discovery:

            try:
                self.discovery.close()

            except Exception:
                pass

            self.discovery = None

        self.browser = None

        self.esp_ip = None

        print("[UDP] Disconnected")

    # ========================================================
    # CONNECTION TIMEOUT
    # ========================================================

    def check_timeout(self, timeout=3.0):

        if not self.connected:
            return

        if self.last_packet_time <= 0:
            return

        elapsed = (
            time.monotonic()
            - self.last_packet_time
        )

        if elapsed > timeout:

            self.set_connected(
                False
            )

            self.set_state(
                "CONNECTION_LOST"
            )

            self.errorOccurred.emit(
                "ESP32 telemetry timeout"
            )


# ---------------------------------------------------------------------------
# Template
# ---------------------------------------------------------------------------

BASE_W = 1920
BASE_H = 1080

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_FILE = os.path.join(
    SCRIPT_DIR, "engine_dashboard_template.png"
)

# If your actual file has another name, change only this line.
if not os.path.exists(TEMPLATE_FILE):
    alternatives = [
        "engine_dashboard_template.png",
        "engine_dashboard_template(1).png",
        "engine_dashboard_template(3).png",
        "a_clean_high_resolution_dark_themed_ui_dashboard.png",
    ]
    for name in alternatives:
        candidate = os.path.join(SCRIPT_DIR, name)
        if os.path.exists(candidate):
            TEMPLATE_FILE = candidate
            break


# ---------------------------------------------------------------------------
# ESP32 UDP / mDNS protocol
# ---------------------------------------------------------------------------
ESP32_MDNS_HOST = "engine-daq.local"
ESP32_UDP_PORT = 4210
GUI_UDP_BIND_HOST = "0.0.0.0"
PROTOCOL_VERSION = 1
TELEMETRY_HZ_EXPECTED = 60
HEARTBEAT_TIMEOUT_S = 2.0
PROFILE_CHUNK_SIZE = 80


def resolve_mdns_host(host=ESP32_MDNS_HOST, timeout=1.0):
    """Resolve an mDNS .local hostname using the OS resolver.

    Windows/macOS/Linux installations with mDNS support resolve this through
    getaddrinfo. The returned IP is used for ordinary UDP; UDP itself does not
    carry mDNS.

    A missing mDNS service should not crash the dashboard; it should simply fall
    back to the known ESP32 IP or wait for discovery.
    """
    old = socket.getdefaulttimeout()
    try:
        socket.setdefaulttimeout(timeout)
        infos = socket.getaddrinfo(host, ESP32_UDP_PORT, socket.AF_INET, socket.SOCK_DGRAM)
        for info in infos:
            addr = info[4][0]
            try:
                ipaddress.ip_address(addr)
                return addr
            except ValueError:
                pass
    except socket.gaierror:
        return None
    except OSError:
        return None
    finally:
        socket.setdefaulttimeout(old)
    return None


# ---------------------------------------------------------------------------
# Normalized coordinate helper
# ---------------------------------------------------------------------------

def scale_rect(base_rect, width, height):
    """
    Convert a rectangle from the 1920 x 1080 template coordinate system
    into the current window coordinate system.
    """
    x, y, w, h = base_rect
    sx = width / BASE_W
    sy = height / BASE_H
    return (
        int(round(x * sx)),
        int(round(y * sy)),
        int(round(w * sx)),
        int(round(h * sy)),
    )


# ---------------------------------------------------------------------------
# Template background widget
# ---------------------------------------------------------------------------

class TemplateBackground(QWidget):
    """
    Paints the supplied template stretched exactly to the full widget.

    Intentionally uses IgnoreAspectRatio so that the image occupies the
    entire application window. This guarantees that normalized overlay
    coordinates stay aligned with the template at every window size.
    """

    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.image_path = image_path
        self.pixmap = QPixmap(image_path)
        self.setAttribute(Qt.WA_StyledBackground, True)

        if self.pixmap.isNull():
            self.pixmap = QPixmap(BASE_W, BASE_H)
            self.pixmap.fill(QColor("#000000"))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        scaled = self.pixmap.scaled(
            self.size(),
            Qt.IgnoreAspectRatio,
            Qt.SmoothTransformation
        )
        painter.drawPixmap(0, 0, scaled)
        painter.end()


# ---------------------------------------------------------------------------
# Transparent overlay layer
# ---------------------------------------------------------------------------

class Overlay(QWidget):
    """
    Transparent layer used for dynamic gauges, bars and status indicators.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WA_NoSystemBackground, True)

        self.rpm = 0.0
        # The supplied template's second gauge is labelled THRUST.
        # UDP still supports the legacy throttle field, while USB/Serial
        # maps Teensy load-cell #1 (10 kg cell) to thrust in grams.
        self.thrust = 0.0
        self.throttle = 0.0
        self.fuel = 0.0
        self.cht = 0.0
        self.egt = 0.0

        self.vib_x = 0.0
        self.vib_y = 0.0
        self.vib_z = 0.0

        self.connected = False
        self.system_ready = False

        # Needle ranges.
        self.rpm_max = 10000
        self.thrust_max = 10000.0
        self.throttle_max = 100.0
        self.fuel_max = 100.0
        self.cht_max = 250
        self.egt_max = 1000

        # Gauge centers in BASE_W x BASE_H coordinates.
        self.gauges = {
            "rpm": {
                "center": (230, 381),
                "radius": 135,
                "min_angle": -135,
                "max_angle": 135,
                "value_box": (151, 444, 156, 64),
            },
            "thrust": {
                "center": (565, 382),
                "radius": 135,
                "min_angle": -135,
                "max_angle": 135,
                "value_box": (482, 446, 160, 64),
            },
            "fuel": {
                "center": (901, 381),
                "radius": 135,
                "min_angle": -135,
                "max_angle": 135,
                "value_box": (822, 444, 158, 64),
            },
            "cht": {
                "center": (420, 709),
                "radius": 135,
                "min_angle": -135,
                "max_angle": 135,
                "value_box": (342, 771, 157, 63),
            },
            "egt": {
                "center": (757, 710),
                "radius": 135,
                "min_angle": -135,
                "max_angle": 135,
                "value_box": (678, 774, 159, 63),
            },
        }

        # Template bars.
        self.bars = {
            "x": (56, 617, 58, 274),
            "y": (128, 617, 58, 274),
            "z": (200, 617, 58, 274),
            "fuel": (936, 617, 59, 274),
        }

    def set_data(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                try:
                    setattr(self, key, float(value))
                except (TypeError, ValueError):
                    pass
        self.update()

    def _xy(self, x, y):
        sx = self.width() / BASE_W
        sy = self.height() / BASE_H
        return QPointF(x * sx, y * sy)

    def _rect(self, rect):
        return QRectF(*scale_rect(
            rect, self.width(), self.height()
        ))

    def _draw_needle(self, painter, gauge, value, maximum):
        cx, cy = gauge["center"]
        radius = gauge["radius"]

        # Map to -135..+135 degrees.
        fraction = max(0.0, min(1.0, value / maximum))
        angle_deg = (
            gauge["min_angle"]
            + fraction * (
                gauge["max_angle"] - gauge["min_angle"]
            )
        )

        # Qt coordinate system: positive Y downward.
        angle = math.radians(angle_deg - 90)
        tip_x = cx + math.cos(angle) * (radius * 0.82)
        tip_y = cy + math.sin(angle) * (radius * 0.82)

        tail_x = cx - math.cos(angle) * (radius * 0.18)
        tail_y = cy - math.sin(angle) * (radius * 0.18)

        pen = QPen(QColor("#ff3030"))
        pen.setWidthF(max(2.5, self.width() / BASE_W * 4.0))
        pen.setCapStyle(Qt.RoundCap)

        painter.setPen(pen)
        painter.drawLine(
            self._xy(tail_x, tail_y),
            self._xy(tip_x, tip_y)
        )

        # Center hub.
        p = self._xy(cx, cy)
        r = 10 * min(
            self.width() / BASE_W,
            self.height() / BASE_H
        )

        painter.setPen(QPen(QColor("#aaaaaa"), 2))
        painter.setBrush(QBrush(QColor("#181818")))
        painter.drawEllipse(p, r, r)

        painter.setBrush(QBrush(QColor("#dddddd")))
        painter.drawEllipse(p, r * 0.25, r * 0.25)

    def _draw_value(self, painter, gauge, text):
        x, y, w, h = gauge["value_box"]
        rect = self._rect((x, y, w, h))

        # Semi-transparent overlay. The template's grey value box remains
        # visible underneath.
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 18, 20, 185))
        painter.drawRoundedRect(
            rect,
            rect.height() * 0.18,
            rect.height() * 0.18
        )

        font = QFont("Arial", max(11, int(self.height() / 929 * 22)))
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor("#f4f4f4"))
        painter.drawText(
            rect,
            Qt.AlignCenter,
            text
        )

    def _draw_bar(self, painter, rect, value, maximum, fill):
        x, y, w, h = rect
        outer = self._rect(rect)

        # Inner bar.
        pad = max(2, int(self.width() / BASE_W * 5))
        inner = QRectF(
            outer.left() + pad,
            outer.top() + pad,
            max(1, outer.width() - 2 * pad),
            max(1, outer.height() - 2 * pad)
        )

        fraction = max(0.0, min(1.0, value / maximum))
        fill_h = inner.height() * fraction

        painter.setPen(Qt.NoPen)

        # Background.
        painter.setBrush(QColor("#535353"))
        painter.drawRoundedRect(
            outer,
            outer.width() * 0.25,
            outer.width() * 0.25
        )

        if fill_h > 0:
            fill_rect = QRectF(
                inner.left(),
                inner.bottom() - fill_h,
                inner.width(),
                fill_h
            )
            painter.setBrush(QColor(fill))
            painter.drawRoundedRect(
                fill_rect,
                inner.width() * 0.22,
                inner.width() * 0.22
            )

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # Gauge needles.
        self._draw_needle(
            painter, self.gauges["rpm"],
            self.rpm, self.rpm_max
        )
        self._draw_needle(
            painter, self.gauges["thrust"],
            self.thrust, self.thrust_max
        )
        self._draw_needle(
            painter, self.gauges["fuel"],
            self.fuel, self.fuel_max
        )
        self._draw_needle(
            painter, self.gauges["cht"],
            self.cht, self.cht_max
        )
        self._draw_needle(
            painter, self.gauges["egt"],
            self.egt, self.egt_max
        )

        # Numeric readouts.
        self._draw_value(
            painter, self.gauges["rpm"],
            f"{int(round(self.rpm))}"
        )
        self._draw_value(
            painter, self.gauges["thrust"],
            f"{self.thrust:.0f} g"
        )
        self._draw_value(
            painter, self.gauges["fuel"],
            f"{self.fuel:.0f} %"
        )
        self._draw_value(
            painter, self.gauges["cht"],
            f"{self.cht:.0f} °C"
        )
        self._draw_value(
            painter, self.gauges["egt"],
            f"{self.egt:.0f} °C"
        )

        # Vibration bars. g values are shown at the top of each bar.
        vib_max = 1.0
        self._draw_bar(
            painter, self.bars["x"],
            abs(self.vib_x), vib_max, "#ff5d7d"
        )
        self._draw_bar(
            painter, self.bars["y"],
            abs(self.vib_y), vib_max, "#4c6fff"
        )
        self._draw_bar(
            painter, self.bars["z"],
            abs(self.vib_z), vib_max, "#b5ff63"
        )
        self._draw_bar(
            painter, self.bars["fuel"],
            self.fuel, 100.0, "#8fe05a"
        )

        # Small vibration numeric values above bars.
        font = QFont("Arial", max(9, int(self.height() / BASE_H * 15)))
        painter.setFont(font)
        painter.setPen(QColor("#eeeeee"))

        for key, value in (
            ("x", self.vib_x),
            ("y", self.vib_y),
            ("z", self.vib_z),
        ):
            x, y, w, h = self.bars[key]
            rect = self._rect((x - 9, y - 39, w + 19, 33))
            painter.drawText(
                rect,
                Qt.AlignCenter,
                f"{value:.2f}"
            )

        # System ready indicator in the open area above the right panel.
        indicator_center = self._xy(773, 141)
        r = max(6, int(self.width() / BASE_W * 10))
        painter.setPen(Qt.NoPen)
        painter.setBrush(
            QColor("#27e957") if self.system_ready
            else QColor("#777777")
        )
        painter.drawEllipse(indicator_center, r, r)

        font = QFont("Arial", max(9, int(self.height() / BASE_H * 15)))
        painter.setFont(font)
        painter.setPen(QColor("#d9d9d9"))
        painter.drawText(
            self._rect((796, 122, 145, 37)),
            Qt.AlignVCenter | Qt.AlignLeft,
            "System Ready" if self.system_ready else "System Offline"
        )

        painter.end()


# ---------------------------------------------------------------------------
# Clickable transparent area helper
# ---------------------------------------------------------------------------

class TransparentButton(QPushButton):
    """
    Invisible click target placed over an element already drawn in the PNG.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFlat(True)
        self.setStyleSheet("""
            QPushButton {
                background: transparent;
                border: none;
            }
            QPushButton:hover {
                background: rgba(255,255,255,20);
                border-radius: 10px;
            }
            QPushButton:pressed {
                background: rgba(255,255,255,35);
            }
        """)


# ---------------------------------------------------------------------------
# UDP receiver
# ---------------------------------------------------------------------------

class UDPReceiver(QObject):
    data_received = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, host="0.0.0.0", port=4210):
        super().__init__()
        self.host = host
        self.port = int(port)
        self.running = False
        self.sock = None

    def run(self):
        self.running = True

        try:
            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )
            self.sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_REUSEADDR,
                1
            )
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.5)

        except Exception as exc:
            self.error.emit(str(exc))
            self.running = False
            return

        while self.running:
            try:
                payload, _addr = self.sock.recvfrom(65535)
                text = payload.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not text:
                    continue

                try:
                    data = json.loads(text)
                except json.JSONDecodeError:
                    data = self._parse_csv_packet(text)

                if isinstance(data, dict):
                    self.data_received.emit(data)

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                self.error.emit(str(exc))

        try:
            self.sock.close()
        except Exception:
            pass

    def _parse_csv_packet(self, text):
        """
        Optional packet format:
        rpm,throttle,fuel,cht,egt,vib_x,vib_y,vib_z
        """
        parts = [p.strip() for p in text.split(",")]

        if len(parts) < 5:
            return None

        names = [
            "rpm", "throttle", "fuel", "cht",
            "egt", "vib_x", "vib_y", "vib_z"
        ]

        result = {}
        for i, value in enumerate(parts[:len(names)]):
            try:
                result[names[i]] = float(value)
            except ValueError:
                pass

        return result

    def stop(self):
        self.running = False

        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# UDP sender
# ---------------------------------------------------------------------------
class UDPSender:
    """Thread-safe UDP command sender for the ESP32."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.target = None

    def set_target(self, host, port=ESP32_UDP_PORT):
        self.target = (host, int(port))

    def send(self, payload):
        if not self.target:
            return False
        try:
            raw = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
            with self.lock:
                self.sock.sendto(raw, self.target)
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main dashboard
# ---------------------------------------------------------------------------

class Dashboard(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ENGINE TEST BENCH DAQ")
        self.setMinimumSize(1280, 720)
        self.setMaximumSize(1920, 1080)

        self.background = None
        self.overlay = None

        # Initialize communication-state defaults before any timers or callbacks can run.
        self.connected = False
        self.last_data_time = None
        self.esp32_ip = None
        self.udp_socket = None
        self.connection_state = "OFFLINE"
        # These are request-in-flight flags, never permanent per-connection latches.
        self.start_sequence_sent = False

        self.udp_thread = None
        self.udp_receiver = None
        self.udp_sender = UDPSender()
        self.udp = ESP32UDPConnection(self)
        self.udp.connectionChanged.connect(self.on_connection_changed)
        self.udp.deviceFound.connect(self.on_device_found)
        self.udp.stateChanged.connect(self.on_esp_state_changed)
        self.udp.telemetryReceived.connect(self.on_telemetry_received)
        self.udp.errorOccurred.connect(self.on_udp_error)
        self.last_state_time = 0
        self.telemetry_count = 0
        self.esp_state = "DISCONNECTED"

        # USB/Serial V1 DAQ state.
        self.serial_thread = None

        self.demo_phase = 0.0

        self._build_ui()
        self._create_click_targets()
        self._create_pages()

        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self._demo_data)
        self.demo_timer.start(100)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._check_data_timeout)
        self.status_timer.start(500)

        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self._serial_timer_tick)
        self.serial_timer.start(20)  # dashboard values ~50 Hz; raw vibration remains buffered at 2 kHz

        print("[DAQ] Starting default USB/Serial connection")
        self.connection_combo.setCurrentIndex(2)
        self.connect_serial()

    # ---------------- Window ----------------

    def _build_ui(self):
        self.background = TemplateBackground(
            TEMPLATE_FILE,
            self
        )
        self.background.setGeometry(self.rect())

        self.overlay = Overlay(self.background)
        self.overlay.setGeometry(self.background.rect())
        self.overlay.raise_()

        # All normal Qt controls sit above the overlay.
        self.connection_combo = QComboBox(self.background)
        self.connection_combo.addItems([
            "Offline / Simulation",
            "ESP32 UDP connection",
            "USB / Serial"
        ])
        self.connection_combo.setCurrentIndex(1)
        self.connection_combo.setStyleSheet("""
            QComboBox {
                background: #d0d0d0;
                color: #111111;
                border: 1px solid #aaaaaa;
                padding: 5px 12px;
                font-size: 18px;
                font-weight: bold;
            }
            QComboBox::drop-down {
                border: 0px;
            }
        """)

        self.connect_btn = QPushButton(
            "CONNECT",
            self.background
        )
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background: #bcbcbc;
                color: #111111;
                border: none;
                font-size: 18px;
                font-weight: bold;
                padding: 5px;
            }
            QPushButton:hover {
                background: #d2d2d2;
            }
        """)
        self.connect_btn.clicked.connect(self.toggle_connection)

        self.start_btn = QPushButton("START", self.background)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background: #1f8f4d;
                color: white;
                border: none;
                font-size: 16px;
                font-weight: bold;
                padding: 5px 12px;
            }
            QPushButton:hover {
                background: #2dad5f;
            }
        """)
        self.start_btn.clicked.connect(self.start_udp_session)

        self.stop_btn = QPushButton("STOP", self.background)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background: #b33636;
                color: white;
                border: none;
                font-size: 16px;
                font-weight: bold;
                padding: 5px 12px;
            }
            QPushButton:hover {
                background: #d54646;
            }
        """)
        self.stop_btn.clicked.connect(self.stop_udp_session)

        # Right-side pages container.
        self.pages = QStackedWidget(self.background)
        self.pages.setStyleSheet("""
            QStackedWidget {
                background: transparent;
                border: none;
            }
        """)

        # Tab buttons.
        self.setup_btn = QPushButton("SETUP", self.background)
        self.exp_btn = QPushButton("EXP", self.background)
        self.analysis_btn = QPushButton("ANALYSIS", self.background)

        for btn in (
            self.setup_btn,
            self.exp_btn,
            self.analysis_btn
        ):
            btn.setStyleSheet("""
                QPushButton {
                    background: #555555;
                    color: white;
                    border: none;
                    border-radius: 10px;
                    font-size: 18px;
                    font-weight: bold;
                    padding: 10px;
                }
                QPushButton:hover {
                    background: #666666;
                }
            """)

        self.setup_btn.clicked.connect(
            lambda: self.pages.setCurrentIndex(0)
        )
        self.exp_btn.clicked.connect(
            lambda: self.pages.setCurrentIndex(1)
        )
        self.analysis_btn.clicked.connect(
            lambda: self.pages.setCurrentIndex(2)
        )

    def _close_analysis_full_page(self):
        self._return_to_dashboard(index=0)

    # ---------------- Click targets ----------------

    def _create_click_targets(self):
        # Live view navigation target.
        self.live_target = TransparentButton(self.background)
        self.live_target.clicked.connect(lambda: self.pages.setCurrentIndex(0))

        # The five template boxes at the bottom are now real DAQ controls.
        # Existing DAQ/servo navigation remains available while the empty
        # boxes gain the requested logging/SD workflow.
        self.daq_target = QPushButton("LIVE / DAQ", self.background)
        self.log_target = QPushButton("START LOG", self.background)
        self.sd_target = QPushButton("SD FILES", self.background)
        self.servo_target = QPushButton("DOWNLOAD", self.background)
        self.sensor_target = QPushButton("SERVO", self.background)

        bottom_style = """
            QPushButton {
                background: #555555;
                color: #ffffff;
                border: 1px solid #666666;
                border-radius: 10px;
                font-size: 15px;
                font-weight: bold;
            }
            QPushButton:hover { background: #686868; }
            QPushButton:pressed { background: #404040; }
        """
        for b in (self.daq_target, self.log_target, self.sd_target,
                  self.servo_target, self.sensor_target):
            b.setStyleSheet(bottom_style)

        self.daq_target.clicked.connect(lambda: self.pages.setCurrentIndex(0))
        self.log_target.clicked.connect(self.start_log_toggle)
        self.sd_target.clicked.connect(lambda: self.open_sd_dialog(False))
        self.servo_target.clicked.connect(lambda: self.open_sd_dialog(True))
        self.sensor_target.clicked.connect(self._show_servo_message)

    # ---------------- Pages ----------------

    def _create_pages(self):
        # Setup.
        if SetupPage is not None:
            try:
                self.setup_page = SetupPage()
                self.pages.addWidget(self.setup_page)
                self.setup_page.configUploaded.connect(
                    self._config_uploaded
                )
            except Exception:
                self.setup_page = self._placeholder_page(
                    "SETUP PAGE ERROR"
                )
                self.pages.addWidget(self.setup_page)
        else:
            self.setup_page = self._placeholder_page(
                "SETUP MODULE NOT FOUND"
            )
            self.pages.addWidget(self.setup_page)

        # Experiment.
        if ExperimentPanel is not None:
            try:
                self.exp_page = ExperimentPanel()
                self.pages.addWidget(self.exp_page)

                # ExperimentPage V1 signals.
                if hasattr(self.exp_page, "runExperiment"):
                    self.exp_page.runExperiment.connect(
                        self._experiment_run
                    )
                if hasattr(self.exp_page, "stopExperiment"):
                    self.exp_page.stopExperiment.connect(
                        self._experiment_stop
                    )
                if hasattr(self.exp_page, "throttleCommand"):
                    self.exp_page.throttleCommand.connect(
                        self._experiment_throttle_command
                    )

                # Compatibility with older experiment implementations.
                if hasattr(self.exp_page, "run_requested"):
                    self.exp_page.run_requested.connect(self._experiment_run)
                if hasattr(self.exp_page, "stop_requested"):
                    self.exp_page.stop_requested.connect(self._experiment_stop)
            except Exception:
                self.exp_page = self._placeholder_page(
                    "EXPERIMENT PAGE ERROR"
                )
                self.pages.addWidget(self.exp_page)
        else:
            self.exp_page = self._placeholder_page(
                "experiment.py NOT FOUND"
            )
            self.pages.addWidget(self.exp_page)

        # Analysis.
        if AnalysisPage is not None:
            try:
                self.analysis_page = AnalysisPage()
                self.pages.addWidget(self.analysis_page)
            except Exception:
                self.analysis_page = self._placeholder_page(
                    "ANALYSIS PAGE ERROR"
                )
                self.pages.addWidget(self.analysis_page)
        else:
            self.analysis_page = self._placeholder_page(
                "analysis_page.py NOT FOUND"
            )
            self.pages.addWidget(self.analysis_page)

    def _placeholder_page(self, text):
        page = QWidget()
        label = QLabel(text)
        label.setStyleSheet(
            "color:#dddddd;font-size:20px;font-weight:bold;"
        )
        layout = QVBoxLayout(page)
        layout.addWidget(label)
        return page

    # ---------------- Resize mapping ----------------

    def resizeEvent(self, event):
        """
        The PNG and all overlays use the same 1650x929 coordinate system.
        """
        # Resize the template canvas itself to exactly the available
        # application client area. All overlay coordinates are then mapped
        # against the same 1920x1080 reference.
        self.background.setGeometry(self.rect())

        if self.overlay:
            self.overlay.setGeometry(
                self.background.rect()
            )
            self.overlay.raise_()

        # Header.
        self.connection_combo.setGeometry(
            *scale_rect(
                (970, 20, 438, 56),
                self.width(),
                self.height()
            )
        )

        self.connect_btn.setGeometry(
            *scale_rect(
                (1431, 20, 372, 56),
                self.width(),
                self.height()
            )
        )
        self.start_btn.setGeometry(
            *scale_rect(
                (1300, 20, 110, 56),
                self.width(),
                self.height()
            )
        )
        self.stop_btn.setGeometry(
            *scale_rect(
                (1180, 20, 110, 56),
                self.width(),
                self.height()
            )
        )

        # Tab buttons from the template.
        self.setup_btn.setGeometry(
            *scale_rect(
                (1104, 160, 159, 69),
                self.width(),
                self.height()
            )
        )

        self.exp_btn.setGeometry(
            *scale_rect(
                (1270, 160, 159, 69),
                self.width(),
                self.height()
            )
        )

        self.analysis_btn.setGeometry(
            *scale_rect(
                (1439, 160, 159, 69),
                self.width(),
                self.height()
            )
        )

        self.pages.setGeometry(
            *scale_rect(
                (1105, 230, 744, 792),
                self.width(),
                self.height()
            )
        )

        # Live view target.
        self.live_target.setGeometry(
            *scale_rect(
                (56, 142, 335, 52),
                self.width(),
                self.height()
            )
        )

        # Bottom status targets.
        self.daq_target.setGeometry(
            *scale_rect(
                (64, 938, 175, 91),
                self.width(),
                self.height()
            )
        )

        self.log_target.setGeometry(
            *scale_rect(
                (256, 938, 175, 91),
                self.width(),
                self.height()
            )
        )

        self.sd_target.setGeometry(
            *scale_rect(
                (442, 938, 175, 91),
                self.width(),
                self.height()
            )
        )

        self.servo_target.setGeometry(
            *scale_rect(
                (634, 938, 175, 91),
                self.width(),
                self.height()
            )
        )

        self.sensor_target.setGeometry(
            *scale_rect(
                (826, 938, 175, 91),
                self.width(),
                self.height()
            )
        )

        self.overlay.raise_()

        # Bring real controls back above overlay.
        for widget in (
            self.connection_combo,
            self.connect_btn,
            self.start_btn,
            self.stop_btn,
            self.pages,
            self.setup_btn,
            self.exp_btn,
            self.analysis_btn,
            self.live_target,
            self.daq_target,
            self.log_target,
            self.sd_target,
            self.servo_target,
            self.sensor_target,
        ):
            widget.raise_()

        event.accept()

    # ---------------- Connection status / protocol ----------------

    def _zero_live_data(self):
        self.overlay.set_data(
            rpm=0.0,
            thrust=0.0,
            throttle=0.0,
            fuel=0.0,
            cht=0.0,
            egt=0.0,
            vib_x=0.0,
            vib_y=0.0,
            vib_z=0.0,
        )
        self.overlay.system_ready = False

    def _set_connection_state(self, state, ready=None):
        prev_state = getattr(self, "connection_state", None)
        prev_ready = getattr(self, "_connection_ready", None)
        next_ready = None if ready is None else bool(ready)

        if prev_state == state and prev_ready == next_ready:
            return

        self.connection_state = state
        self._connection_ready = next_ready
        print(f"[DAQ] connection state -> {state}")
        if ready is not None:
            self.overlay.system_ready = bool(ready)
        colors = {
            "OFFLINE": "#bcbcbc",
            "DISCOVERING": "#f0ad4e",
            "CONNECTING": "#f0ad4e",
            "READY": "#21a842",
            "SERIAL": "#21a842",
            "CONFIGURING": "#1769ff",
            "STARTING": "#f0ad4e",
            "STOPPING": "#f0ad4e",
            "EXPERIMENT": "#1769ff",
            "ERROR": "#c92a3a",
            "STOPPED": "#c92a3a",
        }
        color = colors.get(state, "#bcbcbc")
        self.connect_btn.setText("CONNECTED" if state in ("READY", "SERIAL", "CONFIGURING", "EXPERIMENT") else "CONNECT")
        self.connect_btn.setStyleSheet(f"""
            QPushButton {{ background:{color}; color:white; border:none;
                font-size:18px; font-weight:bold; padding:5px; }}
            QPushButton:hover {{ background:#2fb84b; }}
        """)
        self.overlay.update()

    def send_udp(self, payload):
        """Send one JSON command to the currently resolved ESP32."""
        if hasattr(self, "udp") and self.udp is not None:
            return self.udp.send(payload)
        if not self.esp32_ip:
            return False
        return self.udp_sender.send(payload)

    def on_connection_changed(self, connected):
        self.connected = bool(connected)
        self.overlay.connected = self.connected
        print(f"[DAQ] UDP connected={self.connected}")
        if self.connected:
            self.last_data_time = time.monotonic()
            self._set_connection_state("CONNECTED", True)
        else:
            self.last_data_time = None
            self._zero_live_data()
            self._set_connection_state("OFFLINE", False)

    def on_device_found(self, ip, port):
        self.esp32_ip = str(ip)
        print(f"[DAQ] ESP32 discovered at {ip}:{port}")
        self._set_connection_state("CONNECTED", False)

    def on_esp_state_changed(self, state):
        state_text = str(state or "UNKNOWN").upper()
        if state_text in ("READY", "DAQ_READY", "IDLE"):
            self._set_connection_state("READY", True)
        elif state_text in ("CONFIG_RECEIVED", "CONFIGURING", "CONFIG_UPLOAD"):
            self._set_connection_state("CONFIGURING", False)
        elif state_text in ("RUNNING", "EXPERIMENT_RUNNING"):
            self.start_sequence_sent = False
            self._set_connection_state("EXPERIMENT", True)
        elif state_text in ("STOPPED", "ERROR"):
            if state_text == "STOPPED":
                self.start_sequence_sent = False
            self._set_connection_state(state_text, False)
        elif state_text in ("DISCONNECTED", "OFFLINE"):
            self._set_connection_state("OFFLINE", False)
        else:
            self._set_connection_state("CONNECTED", True)

    def on_telemetry_received(self, data):
        if not isinstance(data, dict):
            return
        print(f"[DAQ] telemetry packet received: {data.get('type', 'telemetry')}")
        self.last_data_time = time.monotonic()
        normalized = self._normalize_data(data)
        if "data" in data and isinstance(data["data"], dict):
            normalized.update(self._normalize_data(data["data"]))
        self.overlay.set_data(**normalized)

    def on_udp_error(self, message):
        print(f"[DAQ] UDP error: {message}")
        self.start_sequence_sent = False
        self._zero_live_data()
        self._set_connection_state("ERROR", False)

    def _send_profile_chunks(self, t, throttle):
        """Send a generated profile as bounded UDP packets."""
        n = min(len(t), len(throttle))
        if n < 2:
            return False
        profile_id = int(time.time() * 1000) & 0xFFFFFFFF
        begin = {
            "type": "profile_begin", "protocol": PROTOCOL_VERSION,
            "profile_id": profile_id, "mode": getattr(self.exp_page, "mode", None).currentText() if hasattr(getattr(self.exp_page, "mode", None), "currentText") else "unknown",
            "points": n, "duration_s": float(t[n-1]),
            "servo_pin": 9
        }
        if not self.send_udp(begin):
            return False
        total = (n + PROFILE_CHUNK_SIZE - 1) // PROFILE_CHUNK_SIZE
        for chunk_index, start in enumerate(range(0, n, PROFILE_CHUNK_SIZE)):
            end = min(start + PROFILE_CHUNK_SIZE, n)
            packet = {
                "type": "profile_chunk", "protocol": PROTOCOL_VERSION,
                "profile_id": profile_id, "chunk": chunk_index, "chunks": total,
                "time_s": [round(float(v), 4) for v in t[start:end]],
                "throttle_percent": [round(float(v), 3) for v in throttle[start:end]]
            }
            if not self.send_udp(packet):
                return False
            time.sleep(0.004)
        return self.send_udp({
            "type": "profile_end", "protocol": PROTOCOL_VERSION,
            "profile_id": profile_id
        })

    # ---------------- Connection ----------------

    def toggle_connection(self):
        if self.connected:
            self.disconnect_udp()
        else:
            self.connect_udp()

    def start_udp_session(self):
        if self.connection_combo.currentText() == "USB / Serial":
            self.start_log_toggle()
            return
        if not self.connected:
            print("[DAQ] START ignored: not connected")
            return
        if self.connection_state in ("STARTING", "EXPERIMENT", "STOPPING"):
            print(f"[DAQ] START ignored: DAQ state is {self.connection_state}")
            return
        if self.start_sequence_sent:
            print("[DAQ] DAQ_START acknowledgement pending")
            return
        self.start_sequence_sent = True
        payload = {"type": "DAQ_START"}
        print("[DAQ] START button pressed")
        ok = self.send_udp(payload)
        if not ok:
            self.start_sequence_sent = False
            print("[DAQ] DAQ_START send failed")
            return
        self.last_data_time = time.monotonic()
        self._set_connection_state("STARTING", False)

    def stop_udp_session(self):
        if self.connection_combo.currentText() == "USB / Serial":
            if logging_active:
                stop_teensy_log()
                return
            print("[DAQ] USB STOP ignored: no active log")
            return
        if not self.connected:
            print("[DAQ] STOP ignored: not connected")
            return
        if self.connection_state == "STOPPING":
            print("[DAQ] STOP acknowledgement pending")
            return
        payload = {"type": "STOP"}
        print("[DAQ] STOP button pressed")
        ok = self.send_udp(payload)
        if not ok:
            print("[DAQ] STOP send failed")
            return
        self._set_connection_state("STOPPING", False)

    def _try_direct_esp32_connect(self):
        """Fallback path used when mDNS discovery is unavailable or blocked."""
        candidates = []
        resolved_ip = resolve_mdns_host()
        if resolved_ip:
            candidates.append(resolved_ip)

        env_ip = os.getenv("ESP32_DAQ_IP")
        if env_ip:
            candidates.append(env_ip.strip())

        # This board is known to advertise itself on this network.
        candidates.extend([
            "10.0.2.245",
            "engine-daq.local",
        ])

        seen = set()
        for candidate in candidates:
            candidate = str(candidate).strip()
            if not candidate or candidate in seen:
                continue
            seen.add(candidate)

            try:
                if candidate.endswith(".local"):
                    ip = resolve_mdns_host(candidate)
                else:
                    ip = candidate
                if not ip:
                    continue
                print(f"[DAQ] Trying direct ESP32 UDP connection to {ip}:{ESP32_UDP_PORT}")
                if self.udp.connect_ip(ip, ESP32_UDP_PORT):
                    self.esp32_ip = str(ip)
                    self._set_connection_state("CONNECTING", False)
                    return True
            except Exception as exc:
                print(f"[DAQ] direct connect failed for {candidate}: {exc}")

        return False

    def connect_udp(self):
        mode = self.connection_combo.currentText()
        print(f"[DAQ] connect_udp() called, mode={mode}")

        if mode == "Offline / Simulation":
            self.stop_serial_connection()
            self.connected = False
            self.overlay.connected = False
            self.esp32_ip = None
            self._zero_live_data()
            self._set_connection_state("OFFLINE", False)
            return

        if mode == "USB / Serial":
            self.connect_serial()
            return

        self.stop_serial_connection()
        self._zero_live_data()
        self._set_connection_state("DISCOVERING", False)
        QApplication.processEvents()

        self.connected = False
        self.overlay.connected = False
        self.esp32_ip = None
        self.last_data_time = None
        self.start_sequence_sent = False
        self.overlay.update()

        try:
            self.udp.discover()
        except Exception as exc:
            print(f"[DAQ] mDNS discovery exception: {exc}")

        direct_ok = self._try_direct_esp32_connect()
        if not direct_ok:
            print("[DAQ] mDNS discovery and direct IP fallback both failed")
            return

        self.connected = True
        self.overlay.connected = True
        self.last_data_time = time.monotonic()
        self._set_connection_state("CONNECTED", True)



    # ---------------- USB / Teensy Serial ----------------

    def connect_serial(self):
        """Start the V1 Teensy binary/text serial receiver on demand."""
        global running, serial_thread

        # The V1 reader uses `running` as its application lifetime flag.
        # Keep it true whenever the dashboard is alive.
        running = True

        if serial_thread is not None and serial_thread.is_alive():
            self.connected = True
            self.overlay.connected = True
            self._set_connection_state("SERIAL", True)
            return

        self._zero_live_data()
        self.overlay.connected = False
        self._set_connection_state("CONNECTING", False)

        serial_thread = threading.Thread(
            target=serial_reader,
            name="TeensySerialReader",
            daemon=True,
        )
        serial_thread.start()

        # The serial reader opens asynchronously.
        QTimer.singleShot(250, self._finish_serial_connect)

    def _finish_serial_connect(self):
        if serial_connected:
            self.connected = True
            self.overlay.connected = True
            self.last_data_time = time.monotonic()
            self._set_connection_state("SERIAL", True)
            print(f"[DAQ] Teensy USB/Serial connected: {SERIAL_PORT} @ {BAUD_RATE}")
        else:
            self.connected = False
            self.overlay.connected = False
            self._set_connection_state("ERROR", False)
            print(f"[DAQ] Could not connect to Teensy on {SERIAL_PORT}")

    def stop_serial_connection(self):
        """Stop the current Teensy serial stream without stopping the GUI."""
        global serial_command_serial, running
        running = False

        try:
            with serial_command_lock:
                ser = serial_command_serial
                serial_command_serial = None

            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        except Exception as exc:
            print("[DAQ] Serial disconnect error:", exc)

        self.connected = False
        self.overlay.connected = False
        self._zero_live_data()
        self.last_data_time = None
        self._set_connection_state("OFFLINE", False)

    def _update_serial_dashboard_data(self):
        """Copy V1 Teensy data into the dashboard gauges and vibration bars."""
        if not serial_connected:
            return

        with data_lock:
            current_load1 = float(load1)
            current_load2 = float(load2)
            current_temp1 = float(temp1)
            current_temp2 = float(temp2)

            # Use the most recent vibration sample for the main gauges/bars.
            current_vx = float(vib_x[-1]) if vib_x else 0.0
            current_vy = float(vib_y[-1]) if vib_y else 0.0
            current_vz = float(vib_z[-1]) if vib_z else 0.0

        # V1 Teensy labels:
        #   Load cell #1 = 10 kg -> thrust gauge, grams
        #   MAX31856 #1 -> EGT
        #   MAX31856 #2 -> CHT
        # RPM and fuel are not present in the V1 serial protocol.
        self.overlay.set_data(
            thrust=current_load1,
            egt=current_temp1 if current_temp1 == current_temp1 else 0.0,
            cht=current_temp2 if current_temp2 == current_temp2 else 0.0,
            vib_x=current_vx,
            vib_y=current_vy,
            vib_z=current_vz,
        )
        self.last_data_time = time.monotonic()
        self.telemetry_count = total_samples

    def _serial_timer_tick(self):
        if self.connection_combo.currentText() == "USB / Serial":
            if serial_connected:
                self._update_serial_dashboard_data()
                if self.connection_state != "SERIAL":
                    self._set_connection_state("SERIAL", True)
            elif self.connection_state == "SERIAL":
                self.connected = False
                self.overlay.connected = False
                self._set_connection_state("ERROR", False)
        self.log_target.setText("STOP LOG" if logging_active else "START LOG")


    def disconnect_udp(self):
        print("[DAQ] disconnect_udp() called")
        if self.connection_combo.currentText() == "USB / Serial":
            self.stop_serial_connection()
            return
        try:
            self.send_udp({"type":"stop", "protocol":PROTOCOL_VERSION, "throttle_percent":0})
        except Exception:
            pass
        if hasattr(self, "udp") and self.udp is not None:
            self.udp.disconnect()
        self.connected = False
        self.overlay.connected = False
        self._zero_live_data()
        self.esp32_ip = None
        self.last_data_time = None
        self._set_connection_state("OFFLINE", False)

        self.connect_btn.setText("CONNECT")
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background:#bcbcbc;
                color:#111111;
                border:none;
                font-size:18px;
                font-weight:bold;
                padding:5px;
            }
        """)

    def _udp_data(self, data):
        now = time.time()
        self.last_data_time = now
        self.telemetry_count += 1

        packet_type = str(data.get("type", data.get("packet", "telemetry"))).lower()
        state = str(data.get("state", data.get("status", ""))).upper()

        if packet_type in ("hello_ack", "hello_response", "hello", "ready"):
            self._set_connection_state("READY", True)
        elif packet_type in ("state", "status") and state:
            if state in ("READY", "DAQ_READY", "IDLE"):
                self._set_connection_state("READY", True)
            elif state in ("RUNNING", "EXPERIMENT_RUNNING"):
                self._set_connection_state("EXPERIMENT", True)
            elif state in ("STOPPED", "ERROR"):
                self._set_connection_state(state, False)
            elif state in ("CONFIGURING", "CONFIG_UPLOAD"):
                self._set_connection_state("CONFIGURING", False)
        elif packet_type in ("setup_ack", "config_ack"):
            self._set_connection_state("READY", True)
        elif packet_type in ("profile_ack", "profile_ready"):
            self._set_connection_state("READY", True)

        normalized = self._normalize_data(data)
        self.overlay.set_data(**normalized)

        # ESP32 can report its actual UDP source address. Keep the resolved
        # mDNS target unless the packet explicitly supplies a usable address.
        remote_ip = data.get("ip") or data.get("device_ip")
        if remote_ip:
            self.esp32_ip = str(remote_ip)


    def _normalize_data(self, data):
        aliases = {
            "rpm": [
                "rpm", "engine_rpm", "RPM"
            ],
            "thrust": [
                "thrust", "thrust_g", "load1", "load_cell_1",
                "force", "thrust_grams"
            ],
            "throttle": [
                "throttle", "throttle_percent",
                "throttle_pct", "servo", "servo_percent"
            ],
            "fuel": [
                "fuel", "fuel_percent",
                "fuel_pct", "fuel_level"
            ],
            "cht": [
                "cht", "cht_c", "cylinder_head_temp"
            ],
            "egt": [
                "egt", "egt_c", "exhaust_gas_temp"
            ],
            "vib_x": [
                "vib_x", "vibration_x", "accel_x", "x"
            ],
            "vib_y": [
                "vib_y", "vibration_y", "accel_y", "y"
            ],
            "vib_z": [
                "vib_z", "vibration_z", "accel_z", "z"
            ],
        }

        out = {}

        lower = {
            str(k).lower(): v
            for k, v in data.items()
        }

        for target, candidates in aliases.items():
            for candidate in candidates:
                if candidate.lower() in lower:
                    out[target] = lower[candidate.lower()]
                    break

        return out

    def _udp_error(self, message):
        # Keep UI alive; demo values continue.
        self.overlay.system_ready = False

    # ---------------- Demo ----------------

    def _demo_data(self):
        """
        Demo values are disabled for the live UDP mode. If the DAQ is not connected,
        the gauges must remain at zero until telemetry arrives.
        """
        if getattr(self, "connected", False):
            return

        mode = self.connection_combo.currentText()
        if mode in ("Offline / Simulation", "USB / Serial"):
            self.overlay.system_ready = False
            self._zero_live_data()
            return

        self._zero_live_data()
        return

    def _check_data_timeout(self):
        if (not getattr(self, "connected", False)
                or self.connection_state not in ("STARTING", "EXPERIMENT")):
            return
        if self.last_data_time is None:
            return
        if time.monotonic() - self.last_data_time > HEARTBEAT_TIMEOUT_S:
            print(f"[DAQ] telemetry timeout: no packet for {HEARTBEAT_TIMEOUT_S}s (socket still connected, waiting for Teensy telemetry)")
            self.start_sequence_sent = False
            self.overlay.system_ready = False
            self._set_connection_state("ERROR", False)

    # ---------------- Config / experiment ----------------

    def _config_uploaded(self, config):
        """
        Called by setup_page.py.

        The same configuration can be forwarded to ESP32 later.
        """
        self._set_connection_state("CONFIGURING", False)
        payload = {
            "type": "setup_upload",
            "command": "UPLOAD_CONFIG",
            "protocol": PROTOCOL_VERSION,
            "config": config,
            "servo_pin": 9,
            "telemetry_hz": TELEMETRY_HZ_EXPECTED
        }
        if self.connected:
            if not self.udp.upload_config(config):
                self._set_connection_state("ERROR", False)
                return
        else:
            self._set_connection_state("OFFLINE", False)

    def _experiment_run(self):
        """
        Called when the experiment panel presses RUN.
        """
        try:
            if not self.connected:
                QMessageBox.warning(self, "ESP32 not connected",
                                    "Connect to the ESP32 before running the experiment.")
                return
            if hasattr(self.exp_page, "get_profile"):
                t, throttle = self.exp_page.get_profile()
                self._set_connection_state("EXPERIMENT", True)
                if not self._send_profile_chunks(t, throttle):
                    raise RuntimeError("Failed to send the throttle profile to ESP32")
                self.send_udp({"type":"profile_start", "protocol":PROTOCOL_VERSION, "servo_pin":9})
        except Exception as exc:
            print("Experiment error:", exc)
            self._set_connection_state("ERROR", False)
            QMessageBox.critical(self, "Experiment communication error", str(exc))

    def _experiment_throttle_command(self, throttle):
        """Forward live throttle commands generated by ExperimentPage."""
        value = max(0.0, min(100.0, float(throttle)))
        self.overlay.throttle = value
        self.overlay.update()

        if self.connection_combo.currentText() == "USB / Serial":
            # The V1 Teensy command path remains available. If the Teensy
            # firmware accepts a throttle command, this forwards it directly.
            send_teensy_command(f"SET_THROTTLE {value:.2f}")
            return

        if self.connected:
            self.send_udp({
                "type": "set_throttle",
                "command": "SET_THROTTLE",
                "protocol": PROTOCOL_VERSION,
                "throttle_percent": value,
                "servo_pin": 9,
            })

    def _experiment_stop(self):
        """
        Emergency/normal stop hook for the experiment planner.
        """
        self.send_udp({
            "type": "stop", "command": "STOP", "protocol": PROTOCOL_VERSION,
            "throttle_percent": 0.0, "servo_pin": 9
        })
        self.overlay.throttle = 0
        self._set_connection_state("READY", True)
        self.overlay.update()

    def start_log_toggle(self):
        if self.connection_combo.currentText() != "USB / Serial":
            QMessageBox.information(
                self, "USB/Serial logging",
                "Select USB / Serial to control the Teensy SD logger from this button."
            )
            return
        if not serial_connected:
            QMessageBox.warning(self, "Teensy disconnected", "Connect the Teensy USB serial port first.")
            return
        if logging_active:
            ok, msg = stop_teensy_log()
            if not ok:
                QMessageBox.warning(self, "STOP LOG", msg)
            return
        filename = self._ask_log_filename()
        if filename:
            ok, msg = start_teensy_log(filename)
            if not ok:
                QMessageBox.warning(self, "START LOG", msg)

    def _ask_log_filename(self):
        default = LOG_FILENAME_DEFAULT
        text, ok = QtWidgets.QInputDialog.getText(
            self, "Start SD Log", "Filename (without .bin is OK):", text=default
        )
        if not ok:
            return ""
        text = text.strip()
        if not text or any(c in text for c in "\\/\r\n"):
            QMessageBox.warning(self, "Invalid filename", "Use a simple root-level filename.")
            return ""
        return text

    def open_sd_dialog(self, direct_download=False):
        if self.connection_combo.currentText() != "USB / Serial" or not serial_connected:
            QMessageBox.warning(self, "Teensy disconnected", "Connect the Teensy USB serial port first.")
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("TEENSY SD CARD")
        dialog.resize(620, 470)
        layout = QVBoxLayout(dialog)

        title = QLabel("Files on Teensy 4.1 SD card (.bin)")
        title.setStyleSheet("font-size:18px;font-weight:bold;")
        layout.addWidget(title)

        file_list = QListWidget()
        file_list.setSelectionMode(QAbstractItemView.SingleSelection)
        layout.addWidget(file_list)

        info = QLabel("Ready")
        layout.addWidget(info)

        progress = QProgressBar()
        progress.setRange(0, 100)
        progress.setValue(0)
        layout.addWidget(progress)

        row = QHBoxLayout()
        refresh = QPushButton("REFRESH")
        download = QPushButton("DOWNLOAD SELECTED")
        close_btn = QPushButton("CLOSE")
        row.addWidget(refresh)
        row.addWidget(download)
        row.addWidget(close_btn)
        layout.addLayout(row)

        def refresh_list():
            ok, msg = request_sd_list()
            info.setText(msg if ok else msg)
            file_list.clear()
            if not ok:
                return
            # Poll briefly without blocking the GUI.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                QApplication.processEvents()
                files, active, complete, error = get_sd_list_state()
                if error:
                    info.setText(error)
                    return
                if complete:
                    for name, size in files:
                        item = QtWidgets.QListWidgetItem(f"{name}    ({size:,} bytes)")
                        item.setData(Qt.UserRole, name)
                        file_list.addItem(item)
                    info.setText(f"{len(files)} file(s) found")
                    if files:
                        file_list.setCurrentRow(0)
                    return
                time.sleep(0.01)
            info.setText("SD_LIST timed out")

        def do_download():
            item = file_list.currentItem()
            if item is None:
                QMessageBox.information(dialog, "Select file", "Select a .bin file first.")
                return
            filename = str(item.data(Qt.UserRole))
            target, _ = QFileDialog.getSaveFileName(
                dialog, "Save Teensy SD Binary", filename,
                "Binary files (*.bin);;All files (*)"
            )
            if not target:
                return
            ok, msg = request_sd_download(filename, target)
            info.setText(msg)
            if not ok:
                return
            download.setEnabled(False)
            refresh.setEnabled(False)
            progress.setValue(0)
            # Non-blocking transfer monitor.
            timer = QTimer(dialog)
            timer.setInterval(100)
            def poll_download():
                with download_lock:
                    active = download_active
                    done = download_completed
                    err = download_error
                    received = download_received_size
                    expected = download_expected_size
                    status = download_last_status
                info.setText(err or status)
                if expected:
                    progress.setValue(min(100, int(received * 100 / expected)))
                if err or done or not active:
                    timer.stop()
                    download.setEnabled(True)
                    refresh.setEnabled(True)
                    if done:
                        progress.setValue(100)
            timer.timeout.connect(poll_download)
            timer.start()

        refresh.clicked.connect(refresh_list)
        download.clicked.connect(do_download)
        close_btn.clicked.connect(dialog.accept)

        refresh_list()
        if direct_download:
            # The same dialog is used by the DOWNLOAD bottom button; the
            # selected first file is downloaded after the user confirms.
            dialog.setWindowTitle("DOWNLOAD TEENSY SD FILE")

        dialog.exec_()

    def _show_servo_message(self):
        QMessageBox.information(
            self,
            "Throttle Servo",
            "Servo control is ready.\n\n"
            "Throttle / thrust command output uses ESP32 GPIO 9.\n"
            "The Experiment tab sends the generated throttle profile "
            "to the ESP32 over UDP."
        )

    # ---------------- Close ----------------

    def closeEvent(self, event):
        try:
            self.send_udp({"type":"stop", "protocol":PROTOCOL_VERSION, "throttle_percent":0})
        except Exception:
            pass
        self.disconnect_udp()
        self.stop_serial_connection()
        running = False
        self.udp_sender.close()
        event.accept()



# Merged-module aliases used by the dashboard.
ExperimentPanel = ExperimentPage



# Active V1 Teensy reader thread. The original V1 reader remains the parser of
# record, so its proven AA55/CRC/text/SD-download protocol is preserved.
serial_thread = None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    app = QApplication([])

    app.setStyle("Fusion")

    dashboard = Dashboard()

    # Start at the native template size. The user can resize the window
    # between the lower and upper limits above.
    dashboard.resize(BASE_W, BASE_H)
    dashboard.show()

    app.exec_()


if __name__ == "__main__":
    main()