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
