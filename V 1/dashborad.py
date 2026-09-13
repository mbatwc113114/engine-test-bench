"""
dashboard.py
Engine Test Bench DAQ Dashboard
--------------------------------

Uses the supplied PNG as a fixed visual template and maps live controls
on top of it using normalized coordinates. The complete window is filled
by the template, so resizing the window keeps the overlay positions aligned.

Expected files in the same folder:
    dashboard.py
    experiment.py
    setup_page.py
    analysis_page.py
    engine_dashboard_template(2).png

Install:
    pip install PyQt5 pyqtgraph pandas numpy

Run:
    python dashboard.py

Notes
-----
The template's native size is 1920 x 1080. All dashboard overlay coordinates
below are expressed in that coordinate system. They are automatically scaled
to the current window size.

The supplied template already contains the visual gauge faces, labels,
top bar, right panel and bottom status panel. This code adds:
    - live gauge needles
    - live numeric values
    - vibration bars
    - fuel level
    - right-side Setup / Exp / Analysis pages
    - UDP connect/disconnect
    - simulated live data when UDP is not connected

Expected ESP32 UDP telemetry JSON example:
{
    "rpm": 5230,
    "throttle": 80,
    "fuel": 65,
    "cht": 128,
    "egt": 632,
    "vib_x": 0.12,
    "vib_y": 0.08,
    "vib_z": 0.15
}

CSV/telemetry aliases are also tolerated.
"""

import json
import math
import os
import socket
import threading
import time
import ipaddress

from udp_connection import ESP32UDPConnection

from PyQt5.QtCore import (
    Qt, QTimer, QRectF, QPointF, pyqtSignal, QObject, QThread
)
from PyQt5.QtGui import (
    QPixmap, QPainter, QPen, QBrush, QColor, QFont, QTransform
)
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox,
    QStackedWidget, QVBoxLayout, QHBoxLayout, QMessageBox,
    QSizePolicy
)

# Optional page modules.
try:
    from setup_page import SetupPage
except Exception:
    SetupPage = None

try:
    from experiment import ExperimentPanel
except Exception:
    ExperimentPanel = None

try:
    from analysis_page import AnalysisPage
except Exception:
    AnalysisPage = None


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
        self.throttle_max = 100
        self.fuel_max = 100
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
            "throttle": {
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
            painter, self.gauges["throttle"],
            self.throttle, self.throttle_max
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
            painter, self.gauges["throttle"],
            f"{self.throttle:.0f} %"
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

        print("[DAQ] Starting default UDP connection attempt")
        self.connection_combo.setCurrentIndex(1)
        self.connect_udp()

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
        # Transparent target over LIVE VIEW title.
        self.live_target = TransparentButton(self.background)
        self.live_target.clicked.connect(
            lambda: self.pages.setCurrentIndex(0)
        )

        # Bottom status targets.
        self.daq_target = TransparentButton(self.background)
        self.log_target = TransparentButton(self.background)
        self.sd_target = TransparentButton(self.background)
        self.servo_target = TransparentButton(self.background)
        self.sensor_target = TransparentButton(self.background)

        self.daq_target.clicked.connect(
            lambda: self.pages.setCurrentIndex(0)
        )

        self.servo_target.clicked.connect(
            self._show_servo_message
        )

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

                # New experiment.py signals.
                if hasattr(self.exp_page, "run_requested"):
                    self.exp_page.run_requested.connect(
                        self._experiment_run
                    )
                if hasattr(self.exp_page, "stop_requested"):
                    self.exp_page.stop_requested.connect(
                        self._experiment_stop
                    )
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
        self.connection_state = state
        print(f"[DAQ] connection state -> {state}")
        if ready is not None:
            self.overlay.system_ready = bool(ready)
        colors = {
            "OFFLINE": "#bcbcbc",
            "DISCOVERING": "#f0ad4e",
            "CONNECTING": "#f0ad4e",
            "READY": "#21a842",
            "CONFIGURING": "#1769ff",
            "EXPERIMENT": "#1769ff",
            "ERROR": "#c92a3a",
            "STOPPED": "#c92a3a",
        }
        color = colors.get(state, "#bcbcbc")
        self.connect_btn.setText("CONNECTED" if state in ("READY", "CONFIGURING", "EXPERIMENT") else "CONNECT")
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
            self._set_connection_state("EXPERIMENT", True)
        elif state_text in ("STOPPED", "ERROR"):
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
        if not self.connected:
            print("[DAQ] START ignored: not connected")
            return
        if self.start_sequence_sent:
            print("[DAQ] DAQ_START already sent for this connection")
            return
        self.start_sequence_sent = True
        print("[DAQ] sending DAQ_START to ESP32")
        self.send_udp({"type": "DAQ_START", "command": "DAQ_START"})
        self.last_data_time = time.monotonic()
        self._set_connection_state("READY", True)

    def stop_udp_session(self):
        if not self.connected:
            print("[DAQ] STOP ignored: not connected")
            return
        print("[DAQ] sending STOP to ESP32")
        self.send_udp({"type": "DAQ_STOP", "command": "STOP"})
        self._set_connection_state("STOPPED", False)

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

        if mode in ("Offline / Simulation", "USB / Serial"):
            self.connected = False
            self.overlay.connected = False
            self.esp32_ip = None
            self._zero_live_data()
            self._set_connection_state("OFFLINE", False)
            return

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

        # After a real socket is established, trigger the DAQ start sequence.
        # Without this, the ESP32 stays in READY/IDLE and never sends telemetry.
        QTimer.singleShot(300, self.start_udp_session)

    def disconnect_udp(self):
        print("[DAQ] disconnect_udp() called")
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
            "throttle": [
                "throttle", "throttle_percent",
                "throttle_pct", "servo", "servo_percent",
                "thrust", "thrust_percent"
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
        if not getattr(self, "connected", False):
            return
        if self.last_data_time is None:
            return
        if time.time() - self.last_data_time > HEARTBEAT_TIMEOUT_S:
            print(f"[DAQ] telemetry timeout: no packet for {HEARTBEAT_TIMEOUT_S}s (socket still connected, waiting for Teensy telemetry)")
            self.overlay.system_ready = False
            self._set_connection_state("READY", False)

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
        self.udp_sender.close()
        event.accept()


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
