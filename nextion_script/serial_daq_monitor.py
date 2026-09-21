import sys
import serial
import threading
import time
import numpy as np
from collections import deque

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


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

# Teensy SD download binary protocol
SD_DOWNLOAD_MAGIC = b"\xD4\x41\x51\x44"
SD_DOWNLOAD_HEADER_SIZE = 15
SD_DOWNLOAD_CRC_SIZE = 2
SD_DOWNLOAD_TYPE_BEGIN = 0x00
SD_DOWNLOAD_TYPE_DATA = 0x01
SD_DOWNLOAD_TYPE_END = 0x02
SD_DOWNLOAD_TYPE_ERROR = 0xFF

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
    global load1, load2, temp1, temp2, rpm, text_section

    s = line.strip()

    if not s:
        return

    upper = s.upper()

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

    # Weight: xxxx g
    if "WEIGHT:" in upper:
        try:
            value_text = s.split(":", 1)[1]
            value_text = value_text.replace("g", "").strip()
            value = float(value_text)

            with data_lock:
                if text_section == "load1":
                    load1 = value
                elif text_section == "load2":
                    load2 = value

        except ValueError:
            pass

        return

    # Thermocouple: xxxx °C
    if "THERMOCOUPLE:" in upper:
        try:
            value_text = s.split(":", 1)[1]
            value_text = (
                value_text
                .replace("°C", "")
                .replace("C", "")
                .strip()
            )

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
    """Validate one complete Teensy SD-download frame and stream its payload."""
    global download_active
    global download_file_handle
    global download_expected_size
    global download_received_size
    global download_error
    global download_completed
    global download_filename
    global download_last_status
    global download_stream_mode

    if len(frame) < SD_DOWNLOAD_HEADER_SIZE + SD_DOWNLOAD_CRC_SIZE:
        return False

    if frame[:4] != SD_DOWNLOAD_MAGIC:
        return False

    frame_type = frame[4]
    payload_len = frame[5] | (frame[6] << 8)
    offset = int.from_bytes(frame[7:11], "little")
    total_size = int.from_bytes(frame[11:15], "little")
    expected_len = SD_DOWNLOAD_HEADER_SIZE + payload_len + SD_DOWNLOAD_CRC_SIZE

    if len(frame) != expected_len:
        return False

    payload = frame[SD_DOWNLOAD_HEADER_SIZE:SD_DOWNLOAD_HEADER_SIZE + payload_len]
    received_crc = frame[-2] | (frame[-1] << 8)
    calculated_crc = crc16_ccitt(frame[4:-2])

    if received_crc != calculated_crc:
        # A CRC failure means the frame boundary can no longer be trusted.
        # Abort the current transfer instead of continuing to interpret
        # arbitrary file bytes as headers.  The user can simply retry.
        with download_lock:
            download_error = (
                f"SD download CRC error at offset {offset:,}. "
                f"Please retry the download."
            )
            download_active = False
            download_stream_mode = False
            if download_file_handle is not None:
                try:
                    download_file_handle.close()
                except Exception:
                    pass
                download_file_handle = None
        return False

    if frame_type == SD_DOWNLOAD_TYPE_BEGIN:
        with download_lock:
            if download_file_handle is None:
                download_error = "Download target file is not open."
                return True
            download_active = True
            download_expected_size = total_size
            download_received_size = 0
            download_filename = payload.decode("utf-8", errors="replace")
            download_completed = False
            download_stream_mode = True
            download_last_status = f"Downloading {download_filename}..."
        return True

    if frame_type == SD_DOWNLOAD_TYPE_DATA:
        with download_lock:
            if not download_active or download_file_handle is None:
                download_error = "Unexpected SD data frame."
                return True
            if offset != download_received_size:
                download_error = (
                    f"SD download offset error: got {offset}, "
                    f"expected {download_received_size}."
                )
                return True
            try:
                download_file_handle.write(payload)
                download_file_handle.flush()
                download_received_size += payload_len
                download_last_status = (
                    f"Downloading: {download_received_size:,} / "
                    f"{download_expected_size:,} bytes"
                )
            except Exception as exc:
                download_error = f"PC file write error: {exc}"
        return True

    if frame_type == SD_DOWNLOAD_TYPE_END:
        with download_lock:
            if download_file_handle is not None:
                try:
                    download_file_handle.flush()
                    download_file_handle.close()
                except Exception:
                    pass
            download_file_handle = None
            download_active = False
            download_stream_mode = False
            if download_received_size == total_size == download_expected_size:
                download_completed = True
                download_last_status = (
                    f"Download complete: {download_received_size:,} bytes"
                )
            else:
                download_error = (
                    f"Incomplete download: {download_received_size:,} / "
                    f"{total_size:,} bytes"
                )
        return True

    if frame_type == SD_DOWNLOAD_TYPE_ERROR:
        message = payload.decode("utf-8", errors="replace")
        with download_lock:
            download_error = message or "Teensy SD download error."
            download_active = False
            download_stream_mode = False
            if download_file_handle is not None:
                try:
                    download_file_handle.close()
                except Exception:
                    pass
                download_file_handle = None
        return True

    with download_lock:
        download_error = f"Unknown SD download frame type: {frame_type}"
    return True


# ============================================================
# SERIAL BINARY STREAM PARSER
# ============================================================

def serial_reader():

    global serial_connected
    global serial_command_serial
    global running
    global total_samples
    global good_packets
    global bad_packets
    global last_packet_arrival_wall_time
    global latest_sample_time
    global last_packet_sample_count

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=0
        )

        # Important: start at the current stream position.
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

        print("Could not open serial port:")
        print(e)

        serial_connected = False
        return

    rx = bytearray()

    # Text is used only for the slow sensors that the current
    # Teensy code prints every 500 ms.
    text_bytes = bytearray()

    # Sample counter (for diagnostics only; NOT used as the plot timestamp).
    sample_index = 0

    while running:

        try:

            waiting = ser.in_waiting

            if waiting > 0:
                chunk = ser.read(waiting)
                if chunk:
                    rx.extend(chunk)
            else:
                # Sub-millisecond idle sleep keeps serial latency low
                # without consuming an entire CPU core.
                time.sleep(0.0005)

            while len(rx) > 0:

                # ------------------------------------------------
                # SD DOWNLOAD FRAME
                # ------------------------------------------------
                # Before BEGIN, locate the 4-byte download magic.
                # After BEGIN, stay in dedicated binary-download mode
                # and parse ONLY complete download frames.  This is much
                # safer than searching for magic bytes inside arbitrary
                # file data.
                # ------------------------------------------------

                if download_stream_mode:
                    if len(rx) < SD_DOWNLOAD_HEADER_SIZE:
                        break

                    if rx[:4] != SD_DOWNLOAD_MAGIC:
                        # We are already in a binary transfer.  Do not
                        # interpret bytes as ASCII or telemetry. Wait for
                        # the next serial chunk; a malformed stream will
                        # eventually time out at the GUI level.
                        del rx[0]
                        continue

                    payload_len = rx[5] | (rx[6] << 8)
                    frame_size = (
                        SD_DOWNLOAD_HEADER_SIZE
                        + payload_len
                        + SD_DOWNLOAD_CRC_SIZE
                    )

                    if len(rx) < frame_size:
                        break

                    frame = bytes(rx[:frame_size])
                    del rx[:frame_size]
                    handle_sd_download_frame(frame)
                    continue

                download_sync_pos = rx.find(SD_DOWNLOAD_MAGIC)

                if download_sync_pos == 0:
                    if len(rx) < SD_DOWNLOAD_HEADER_SIZE:
                        break

                    payload_len = rx[5] | (rx[6] << 8)
                    frame_size = (
                        SD_DOWNLOAD_HEADER_SIZE
                        + payload_len
                        + SD_DOWNLOAD_CRC_SIZE
                    )

                    if len(rx) < frame_size:
                        break

                    frame = bytes(rx[:frame_size])
                    del rx[:frame_size]
                    handle_sd_download_frame(frame)
                    continue

                # If a download magic occurs before the next telemetry
                # packet, process preceding bytes as normal text.
                if download_sync_pos > 0:
                    telemetry_sync_pos = rx.find(bytes([SYNC1, SYNC2]))
                    if telemetry_sync_pos == -1 or download_sync_pos < telemetry_sync_pos:
                        text_bytes.extend(rx[:download_sync_pos])
                        del rx[:download_sync_pos]
                        while b"\n" in text_bytes:
                            line, _, remainder = text_bytes.partition(b"\n")
                            text_bytes = bytearray(remainder)
                            try:
                                process_text_line(line.decode("utf-8", errors="ignore"))
                            except Exception:
                                pass
                        continue

                # ------------------------------------------------
                # Find binary packet start.
                # ------------------------------------------------

                sync_pos = rx.find(bytes([SYNC1, SYNC2]))

                if sync_pos == -1:

                    # Everything is human-readable text.
                    text_bytes.extend(rx)
                    rx.clear()
                    break

                # Bytes before AA55 are text.
                if sync_pos > 0:

                    text_bytes.extend(rx[:sync_pos])
                    del rx[:sync_pos]

                    # Process complete text lines.
                    while b"\n" in text_bytes:

                        line, _, remainder = text_bytes.partition(b"\n")
                        text_bytes = bytearray(remainder)

                        try:
                            process_text_line(
                                line.decode(
                                    "utf-8",
                                    errors="ignore"
                                )
                            )
                        except Exception:
                            pass

                    continue

                # ------------------------------------------------
                # We are at AA55.
                # Need full 5-byte header.
                # ------------------------------------------------

                if len(rx) < HEADER_SIZE:
                    break

                packet_type = rx[2]

                count = (
                    rx[3]
                    | (rx[4] << 8)
                )

                # Only current acceleration packet is expected.
                if packet_type != PACKET_TYPE_ACCEL:

                    # Drop one byte and search again.
                    del rx[0]
                    continue

                payload_size = count * 6
                packet_size = (
                    HEADER_SIZE
                    + payload_size
                    + CRC_SIZE
                )

                if len(rx) < packet_size:
                    break

                packet = bytes(rx[:packet_size])
                del rx[:packet_size]

                payload = packet[
                    HEADER_SIZE:
                    HEADER_SIZE + payload_size
                ]

                received_crc = (
                    packet[-2]
                    | (packet[-1] << 8)
                )

                calculated_crc = crc16_ccitt(
                    packet[2:-2]
                )

                if received_crc != calculated_crc:

                    bad_packets += 1

                    # CRC failed. Do not use data.
                    continue

                good_packets += 1

                # ------------------------------------------------
                # Record packet timing.
                #
                # The packet contains a block of consecutive 1 kHz
                # samples.  Keep the actual latest sample timestamp,
                # plus the PC wall time when this packet arrived.
                # The GUI uses these to interpolate continuously
                # between 100 packets/s while rendering at 100 Hz.
                # ------------------------------------------------
                packet_arrival_wall_time = time.time()

                # ------------------------------------------------
                # Decode every acceleration sample.
                #
                # IMPORTANT:
                # The current Teensy binary packet does NOT contain
                # a sensor timestamp. Therefore we timestamp the
                # packet at the PC when it arrives and reconstruct
                # the individual sample times backwards at 2 kHz.
                #
                # This is much better for real-time plotting than
                # using sample_index / sample_rate, but it is still
                # an estimated acquisition timestamp.
                # ------------------------------------------------

                global stream_start_wall_time

                # Establish a wall-clock reference once.
                # IMPORTANT: the X spacing is NOT based on USB packet
                # arrival jitter. Every sample is exactly 1/2000 s apart.
                if stream_start_wall_time is None:
                    stream_start_wall_time = (
                        time.time()
                        - (count - 1) / ACC_SAMPLE_RATE_HZ
                    )

                with data_lock:

                    for i in range(count):

                        p = i * 6

                        raw_x = (
                            payload[p]
                            | (payload[p + 1] << 8)
                        )

                        raw_y = (
                            payload[p + 2]
                            | (payload[p + 3] << 8)
                        )

                        raw_z = (
                            payload[p + 4]
                            | (payload[p + 5] << 8)
                        )

                        gx = adc_to_g(
                            raw_x,
                            ADXL_OFFSET_X_V,
                            ADXL_SENS_X_V_PER_G
                        )

                        gy = adc_to_g(
                            raw_y,
                            ADXL_OFFSET_Y_V,
                            ADXL_SENS_Y_V_PER_G
                        )

                        gz = adc_to_g(
                            raw_z,
                            ADXL_OFFSET_Z_V,
                            ADXL_SENS_Z_V_PER_G
                        )

                        # TRUE DISPLAY TIME BASE:
                        # one sample every exactly 0.5 ms.
                        #
                        # This prevents USB packet arrival jitter from
                        # stretching/compressing the X axis.
                        t = (
                            stream_start_wall_time
                            + sample_index
                            / ACC_SAMPLE_RATE_HZ
                        )

                        time_data.append(t)
                        vib_x.append(gx)
                        vib_y.append(gy)
                        vib_z.append(gz)

                        sample_index += 1
                        total_samples += 1

                # Latest actual sample in this packet.
                latest_sample_time = (
                    stream_start_wall_time
                    + (sample_index - 1) / ACC_SAMPLE_RATE_HZ
                )
                last_packet_arrival_wall_time = packet_arrival_wall_time
                last_packet_sample_count = count

            # Process any complete text lines left after binary packets.
            while b"\n" in text_bytes:

                line, _, remainder = text_bytes.partition(b"\n")
                text_bytes = bytearray(remainder)

                try:
                    process_text_line(
                        line.decode(
                            "utf-8",
                            errors="ignore"
                        )
                    )
                except Exception:
                    pass

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

    serial_connected = False
    with serial_command_lock:
        serial_command_serial = None


# ============================================================

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
    global download_file_handle
    global download_active
    global download_target_path
    global download_expected_size
    global download_received_size
    global download_error
    global download_completed
    global download_filename
    global download_last_status
    global download_stream_mode

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
        download_last_status = "Waiting for Teensy..."

    return True, "PC destination ready."


def cancel_sd_download_file():
    global download_file_handle
    global download_active
    global download_stream_mode
    with download_lock:
        download_active = False
        download_stream_mode = False
        if download_file_handle is not None:
            try:
                download_file_handle.close()
            except Exception:
                pass
            download_file_handle = None


def request_sd_download(filename, target_path):
    ok, message = prepare_sd_download(target_path)
    if not ok:
        return False, message

    ok, message = send_teensy_command(f"DOWNLOAD {filename}")
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

class DAQMonitor(QtWidgets.QMainWindow):

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


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    serial_thread = threading.Thread(
        target=serial_reader,
        daemon=True
    )

    serial_thread.start()

    app = QtWidgets.QApplication(sys.argv)

    window = DAQMonitor()
    window.show()

    sys.exit(app.exec_())
