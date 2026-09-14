#!/usr/bin/env python3

"""
=============================================================================
ENGINE TEST BENCH DAQ
Python Master Controller / Engine Simulator
=============================================================================

ARCHITECTURE

    Nextion throtleSlider
            |
            v
          ESP32
            |
            v
         Python
        /      \
       /        \
engine          servo=XX
simulation         |
                   v
                 ESP32
                   |
                   v
                GPIO10
                   |
                   v
                 Servo


PYTHON IS THE MASTER CONTROLLER.

Nextion:
    throtleSlider = operator throttle input

Python:
    reads throttle
    runs engine simulation
    calculates servo angle
    sends servo=XX

ESP32:
    receives servo=XX
    drives GPIO10
    forwards normal Nextion commands
    forwards Nextion responses

IMPORTANT:

    Python NEVER writes:

        throtleSlider.val

SERIAL:

    ESP32 USB COM16
    115200 baud

COMMAND TERMINATION:

    FF FF FF
=============================================================================
"""

import serial
import struct
import time
import random
import math


# =============================================================================
# CONFIGURATION
# =============================================================================

PORT = "COM14"

BAUD = 115200

DEBUG_SERIAL = False


# =============================================================================
# UPDATE RATES
# =============================================================================

ENGINE_HZ = 60

THROTTLE_READ_HZ = 10

DISPLAY_HZ = 10


ENGINE_PERIOD = 1.0 / ENGINE_HZ

THROTTLE_PERIOD = 1.0 / THROTTLE_READ_HZ

DISPLAY_PERIOD = 1.0 / DISPLAY_HZ


# =============================================================================
# NEXTION COMPONENTS
# =============================================================================

NEXTION = {

    "numeric": {

        "rpm": "rpm",

        "thrust": "thrust",

        "fuel": "fule",

        "cht": "cht",

        "egt": "eht"
    },


    "gauges": {

        "rpm": "rpmGauge",

        "thrust": "thrustGauge",

        "fuel": "fuleGauge",

        "cht": "chtGauge",

        "egt": "ehtGauge"
    },


    "throttle": {

        "slider": "throtleSlider",

        "text": "throtle"
    },


    "vibration": {

        "x": "vibX",

        "y": "vibY",

        "z": "vibZ"
    },


    "waveform": {

        "object_id": 2,

        "channel": 0
    }
}


# =============================================================================
# ENGINE PARAMETERS
# =============================================================================

DISPLACEMENT_CC = 60.32

IDLE_RPM = 1500

MAX_RPM = 7000

NOMINAL_FULL_RPM = 6500

MAX_THRUST_KG = 7.0

FUEL_CAPACITY_CC = 300.0

FULL_FUEL_FLOW_CC_MIN = 30.0

PROPELLER_LOAD = 1.0


# =============================================================================
# NEXTION DISPLAY / ESP32 BRIDGE
# =============================================================================

class NextionDisplay:

    def __init__(
        self,
        port,
        baud,
        debug=False
    ):

        self.ser = serial.Serial(

            port=port,

            baudrate=baud,

            bytesize=serial.EIGHTBITS,

            parity=serial.PARITY_NONE,

            stopbits=serial.STOPBITS_ONE,

            timeout=0,

            write_timeout=0.25
        )


        # ESP32 may reset when COM port opens

        time.sleep(
            2.0
        )


        self._rx_buffer = bytearray()

        self.debug = debug


        print(
            f"Connected to {port} at {baud}"
        )


    # =========================================================================
    # SEND COMMAND
    # =========================================================================

    def send(
        self,
        command
    ):

        packet = (

            command.encode(
                "ascii",
                errors="ignore"
            )

            +

            b"\xFF\xFF\xFF"
        )


        self.ser.write(
            packet
        )


        # Small pacing delay
        #
        # Prevents flooding Nextion command queue.

        time.sleep(
            0.002
        )


    # =========================================================================
    # SERVO COMMAND
    # =========================================================================

    def set_servo(
        self,
        angle
    ):

        angle = int(
            round(
                angle
            )
        )


        angle = max(
            0,
            min(
                180,
                angle
            )
        )


        # Python -> ESP32
        #
        # Example:
        #
        # servo=60 FF FF FF

        self.send(
            f"servo={angle}"
        )


    # =========================================================================
    # NEXTION NUMERIC
    # =========================================================================

    def set_numeric(
        self,
        component,
        value
    ):

        self.send(
            f"{component}.val={int(value)}"
        )


    # =========================================================================
    # NEXTION TEXT
    # =========================================================================

    def set_text(
        self,
        component,
        text
    ):

        safe = str(
            text
        ).replace(
            '"',
            "'"
        )


        self.send(
            f'{component}.txt="{safe}"'
        )


    # =========================================================================
    # WAVEFORM
    # =========================================================================

    def add_waveform_point(
        self,
        object_id,
        channel,
        value
    ):

        value = max(
            0,
            min(
                255,
                int(value)
            )
        )


        self.send(
            f"add {object_id},{channel},{value}"
        )


    # =========================================================================
    # REQUEST THROTTLE
    # =========================================================================

    def request_throttle(
        self
    ):

        command = (
            f"get "
            f"{NEXTION['throttle']['slider']}.val"
        )


        self.send(
            command
        )


    # =========================================================================
    # READ SERIAL DATA
    # =========================================================================

    def read_available(
        self
    ):

        # -------------------------------------------------------------
        # Read all currently available bytes
        # -------------------------------------------------------------

        n = self.ser.in_waiting


        if n:

            data = self.ser.read(
                n
            )


            self._rx_buffer.extend(
                data
            )


        values = []


        buf = self._rx_buffer


        i = 0

        consumed_to = 0


        # -------------------------------------------------------------
        # Search for Nextion 0x71 packets
        #
        # Format:
        #
        # 71
        # 4-byte integer
        # FF FF FF
        #
        # Total = 8 bytes
        # -------------------------------------------------------------

        while (
            i < len(buf)
        ):

            if buf[i] == 0x71:

                if (
                    i + 8
                    <= len(buf)
                ):

                    packet = (
                        buf[
                            i:i + 8
                        ]
                    )


                    # Check terminator

                    if (
                        packet[5] == 0xFF
                        and
                        packet[6] == 0xFF
                        and
                        packet[7] == 0xFF
                    ):

                        value = struct.unpack(
                            "<I",
                            packet[1:5]
                        )[0]


                        values.append(
                            value
                        )


                        if self.debug:

                            print(
                                f"[THROTTLE PARSED] "
                                f"value={value}"
                            )


                        i += 8

                        consumed_to = i

                        continue


                    else:

                        i += 1

                        consumed_to = i

                        continue


                else:

                    # Incomplete packet.
                    #
                    # Wait for more bytes.

                    break


            else:

                # Ignore:

                # ESP32 debug text
                # other ASCII
                # other Nextion responses

                i += 1

                consumed_to = i


        # -------------------------------------------------------------
        # Remove already scanned data
        # -------------------------------------------------------------

        if consumed_to > 0:

            del self._rx_buffer[
                :consumed_to
            ]


        # -------------------------------------------------------------
        # Safety buffer limit
        # -------------------------------------------------------------

        if (
            len(self._rx_buffer)
            > 4096
        ):

            self._rx_buffer.clear()


        return values


    # =========================================================================
    # CLOSE
    # =========================================================================

    def close(
        self
    ):

        if self.ser.is_open:

            self.ser.close()


# =============================================================================
# ENGINE SIMULATION
# =============================================================================

class EngineSimulation:

    """
    Saito FG-60R3 / 60cc simplified engine model.

    Inputs:
        throttle 0-100%

    Outputs:
        RPM
        thrust
        fuel flow
        fuel remaining
        CHT
        EGT
        vibration X/Y/Z
    """

    def __init__(self):

        self.throttle = 0.0

        self.rpm = 0.0

        self.thrust = 0.0

        self.fuel_remaining = (
            FUEL_CAPACITY_CC
        )

        self.fuel_flow = 0.0

        self.cht = 25.0

        self.egt = 25.0

        self.vib_x = 0.0

        self.vib_y = 0.0

        self.vib_z = 0.0


        # -------------------------------------------------------------
        # Response time constants
        # -------------------------------------------------------------

        self.tau_rpm = 0.6

        self.tau_thrust = 0.4

        self.tau_cht = 8.0

        self.tau_egt = 3.0


    # =========================================================================
    # TARGET RPM
    # =========================================================================

    def _target_rpm(
        self,
        throttle_pct
    ):

        if throttle_pct <= 0:

            return 0.0


        span = (
            MAX_RPM
            -
            IDLE_RPM
        )


        return (
            IDLE_RPM
            +
            (
                span
                *
                (
                    throttle_pct
                    /
                    100.0
                )
            )
        )


    # =========================================================================
    # TARGET THRUST
    # =========================================================================

    def _target_thrust(
        self,
        rpm
    ):

        if rpm <= IDLE_RPM:

            return 0.0


        frac = (

            rpm
            -
            IDLE_RPM

        ) / (

            NOMINAL_FULL_RPM
            -
            IDLE_RPM
        )


        frac = max(
            0.0,
            min(
                1.0,
                frac
            )
        )


        return (
            MAX_THRUST_KG
            *
            (
                frac
                **
                PROPELLER_LOAD
            )
        )


    # =========================================================================
    # TARGET CHT
    # =========================================================================

    def _target_cht(
        self,
        rpm
    ):

        frac = (
            rpm
            /
            MAX_RPM
        )


        return (
            25.0
            +
            frac * 200.0
        )


    # =========================================================================
    # TARGET EGT
    # =========================================================================

    def _target_egt(
        self,
        rpm
    ):

        frac = (
            rpm
            /
            MAX_RPM
        )


        return (
            25.0
            +
            frac * 700.0
        )


    # =========================================================================
    # ENGINE UPDATE
    # =========================================================================

    def update(
        self,
        throttle_pct,
        dt
    ):

        # -------------------------------------------------------------
        # Throttle safety
        # -------------------------------------------------------------

        self.throttle = max(
            0.0,
            min(
                100.0,
                throttle_pct
            )
        )


        # -------------------------------------------------------------
        # RPM
        # -------------------------------------------------------------

        target_rpm = (
            self._target_rpm(
                self.throttle
            )
        )


        alpha_rpm = (
            1.0
            -
            math.exp(
                -dt
                /
                self.tau_rpm
            )
        )


        self.rpm += (
            target_rpm
            -
            self.rpm
        ) * alpha_rpm


        # -------------------------------------------------------------
        # THRUST
        # -------------------------------------------------------------

        target_thrust = (
            self._target_thrust(
                self.rpm
            )
        )


        alpha_thrust = (
            1.0
            -
            math.exp(
                -dt
                /
                self.tau_thrust
            )
        )


        self.thrust += (
            target_thrust
            -
            self.thrust
        ) * alpha_thrust


        # -------------------------------------------------------------
        # FUEL FLOW
        # -------------------------------------------------------------

        rpm_frac = max(
            0.0,
            min(
                1.0,
                self.rpm
                /
                MAX_RPM
            )
        )


        self.fuel_flow = (
            FULL_FUEL_FLOW_CC_MIN
            *
            rpm_frac
        )


        consumed = (
            self.fuel_flow
            *
            (
                dt
                /
                60.0
            )
        )


        self.fuel_remaining = max(
            0.0,
            self.fuel_remaining
            -
            consumed
        )


        # -------------------------------------------------------------
        # CHT
        # -------------------------------------------------------------

        target_cht = (
            self._target_cht(
                self.rpm
            )
        )


        alpha_cht = (
            1.0
            -
            math.exp(
                -dt
                /
                self.tau_cht
            )
        )


        self.cht += (
            target_cht
            -
            self.cht
        ) * alpha_cht


        # -------------------------------------------------------------
        # EGT
        # -------------------------------------------------------------

        target_egt = (
            self._target_egt(
                self.rpm
            )
        )


        alpha_egt = (
            1.0
            -
            math.exp(
                -dt
                /
                self.tau_egt
            )
        )


        self.egt += (
            target_egt
            -
            self.egt
        ) * alpha_egt


        # -------------------------------------------------------------
        # VIBRATION
        # -------------------------------------------------------------

        vib_base = (
            5.0
            +
            45.0
            *
            rpm_frac
        )


        self.vib_x = max(
            0.0,
            vib_base
            +
            random.uniform(
                -6,
                6
            )
        )


        self.vib_y = max(
            0.0,
            vib_base
            *
            0.7
            +
            random.uniform(
                -5,
                5
            )
        )


        self.vib_z = max(
            0.0,
            vib_base
            *
            0.8
            +
            random.uniform(
                -5,
                5
            )
        )


    # =========================================================================
    # FUEL PERCENT
    # =========================================================================

    @property
    def fuel_percent(
        self
    ):

        return max(
            0.0,
            min(
                100.0,
                (
                    self.fuel_remaining
                    /
                    FUEL_CAPACITY_CC
                )
                *
                100.0
            )
        )


# =============================================================================
# DISPLAY SCALE
# =============================================================================

def scale(
    value,
    in_min,
    in_max,
    out_min,
    out_max
):

    value = max(
        in_min,
        min(
            in_max,
            value
        )
    )


    return (
        out_min
        +
        (
            value
            -
            in_min
        )
        *
        (
            out_max
            -
            out_min
        )
        /
        (
            in_max
            -
            in_min
        )
    )


# =============================================================================
# GAUGE MAPPINGS
# =============================================================================

def map_rpm_gauge(
    rpm
):

    return scale(
        rpm,
        0,
        8000,
        0,
        270
    )


def map_thrust_gauge(
    thrust_kg
):

    return scale(
        thrust_kg,
        0,
        7,
        0,
        270
    )


def map_fuel_gauge(
    fuel_pct
):

    return scale(
        fuel_pct,
        0,
        100,
        0,
        270
    )


def map_cht_gauge(
    cht_c
):

    return scale(
        cht_c,
        0,
        250,
        0,
        270
    )


def map_egt_gauge(
    egt_c
):

    return scale(
        egt_c,
        0,
        800,
        0,
        270
    )


def map_rpm_waveform(
    rpm
):

    return scale(
        rpm,
        0,
        8000,
        0,
        255
    )


# =============================================================================
# MAIN
# =============================================================================

def main():

    print()
    print(
        "=" * 70
    )

    print(
        " ENGINE TEST BENCH DAQ"
    )

    print(
        " Python Master Controller"
    )

    print(
        "=" * 70
    )

    print(
        f"ESP32 PORT : {PORT}"
    )

    print(
        f"BAUD       : {BAUD}"
    )

    print()

    print(
        "CONTROL:"
    )

    print(
        "Nextion slider -> Python -> ESP32 -> Servo GPIO10"
    )

    print()

    print(
        "IMPORTANT:"
    )

    print(
        "Close Arduino Serial Monitor before starting."
    )

    print(
        "=" * 70
    )


    # =========================================================================
    # CONNECT
    # =========================================================================

    try:

        display = NextionDisplay(
            PORT,
            BAUD,
            debug=DEBUG_SERIAL
        )

    except serial.SerialException as e:

        print()

        print(
            f"ERROR: Could not open {PORT}"
        )

        print(
            e
        )

        print()

        print(
            "Make sure:"
        )

        print(
            "1. ESP32 is connected."
        )

        print(
            "2. Serial Monitor is CLOSED."
        )

        print(
            "3. Correct COM port is being used."
        )

        return


    # =========================================================================
    # ENGINE
    # =========================================================================

    engine = EngineSimulation()


    # =========================================================================
    # VARIABLES
    # =========================================================================

    throttle_input = 0.0

    servo_angle = 0

    last_servo_angle = None


    now = time.perf_counter()


    last_engine_update = now

    last_throttle_request = now

    last_display_update = now

    last_telemetry_print = now


    # =========================================================================
    # MAIN LOOP
    # =========================================================================

    try:

        while True:

            now = time.perf_counter()


            # ================================================================
            # 1. READ ALL AVAILABLE NEXTION DATA
            # ================================================================

            values = (
                display.read_available()
            )


            if values:

                # ------------------------------------------------------------
                # Take newest throttle value
                # ------------------------------------------------------------

                throttle_input = float(
                    values[-1]
                )


                # ------------------------------------------------------------
                # Safety clamp
                # ------------------------------------------------------------

                throttle_input = max(
                    0.0,
                    min(
                        100.0,
                        throttle_input
                    )
                )


                # ============================================================
                # THROTTLE -> SERVO
                # ============================================================

                servo_angle = int(
                    round(
                        throttle_input
                        *
                        1.8
                    )
                )


                servo_angle = max(
                    0,
                    min(
                        180,
                        servo_angle
                    )
                )


                # ============================================================
                # SEND SERVO ONLY WHEN IT CHANGES
                # ============================================================

                if (
                    servo_angle
                    !=
                    last_servo_angle
                ):

                    display.set_servo(
                        servo_angle
                    )


                    print(
                        f"[SERVO -> ESP32] "
                        f"Throttle="
                        f"{throttle_input:.1f}% "
                        f"Servo="
                        f"{servo_angle} deg"
                    )


                    last_servo_angle = (
                        servo_angle
                    )


            # ================================================================
            # 2. REQUEST THROTTLE FROM NEXTION
            # ================================================================

            if (
                now
                -
                last_throttle_request
                >=
                THROTTLE_PERIOD
            ):

                display.request_throttle()


                last_throttle_request = now


            # ================================================================
            # 3. ENGINE PHYSICS
            # ================================================================

            if (
                now
                -
                last_engine_update
                >=
                ENGINE_PERIOD
            ):

                dt = (
                    now
                    -
                    last_engine_update
                )


                dt = max(
                    0.0,
                    min(
                        dt,
                        0.2
                    )
                )


                engine.update(
                    throttle_input,
                    dt
                )


                last_engine_update = now


            # ================================================================
            # 4. UPDATE NEXTION DISPLAY
            # ================================================================

            if (
                now
                -
                last_display_update
                >=
                DISPLAY_PERIOD
            ):

                # ------------------------------------------------------------
                # Numeric values
                # ------------------------------------------------------------

                display.set_numeric(
                    NEXTION["numeric"]["rpm"],
                    engine.rpm
                )


                display.set_numeric(
                    NEXTION["numeric"]["thrust"],
                    engine.thrust * 100
                )


                display.set_numeric(
                    NEXTION["numeric"]["fuel"],
                    engine.fuel_percent
                )


                display.set_numeric(
                    NEXTION["numeric"]["cht"],
                    engine.cht
                )


                display.set_numeric(
                    NEXTION["numeric"]["egt"],
                    engine.egt
                )


                # ------------------------------------------------------------
                # Throttle text
                #
                # IMPORTANT:
                #
                # We update:
                #
                #     throtle.txt
                #
                # NOT:
                #
                #     throtleSlider.val
                # ------------------------------------------------------------

                display.set_text(
                    NEXTION["throttle"]["text"],
                    f"{throttle_input:.0f}%"
                )


                # ------------------------------------------------------------
                # Gauges
                # ------------------------------------------------------------

                display.set_numeric(
                    NEXTION["gauges"]["rpm"],
                    map_rpm_gauge(
                        engine.rpm
                    )
                )


                display.set_numeric(
                    NEXTION["gauges"]["thrust"],
                    map_thrust_gauge(
                        engine.thrust
                    )
                )


                display.set_numeric(
                    NEXTION["gauges"]["fuel"],
                    map_fuel_gauge(
                        engine.fuel_percent
                    )
                )


                display.set_numeric(
                    NEXTION["gauges"]["cht"],
                    map_cht_gauge(
                        engine.cht
                    )
                )


                display.set_numeric(
                    NEXTION["gauges"]["egt"],
                    map_egt_gauge(
                        engine.egt
                    )
                )


                # ------------------------------------------------------------
                # VIBRATION
                # ------------------------------------------------------------

                display.set_numeric(
                    NEXTION["vibration"]["x"],
                    engine.vib_x
                )


                display.set_numeric(
                    NEXTION["vibration"]["y"],
                    engine.vib_y
                )


                display.set_numeric(
                    NEXTION["vibration"]["z"],
                    engine.vib_z
                )


                # ------------------------------------------------------------
                # RPM WAVEFORM
                # ------------------------------------------------------------

                display.add_waveform_point(
                    NEXTION["waveform"]["object_id"],
                    NEXTION["waveform"]["channel"],
                    map_rpm_waveform(
                        engine.rpm
                    )
                )


                last_display_update = now


            # ================================================================
            # 5. CONSOLE TELEMETRY
            # ================================================================

            if (
                now
                -
                last_telemetry_print
                >=
                1.0
            ):

                servo_angle_display = (
                    throttle_input
                    *
                    1.8
                )


                print(
                    f"THR="
                    f"{throttle_input:5.1f}% | "

                    f"SERVO="
                    f"{servo_angle_display:6.1f} deg | "

                    f"RPM="
                    f"{engine.rpm:5.0f} | "

                    f"THRUST="
                    f"{engine.thrust:4.2f} kgf | "

                    f"FLOW="
                    f"{engine.fuel_flow:5.1f} cc/min | "

                    f"FUEL="
                    f"{engine.fuel_percent:5.1f}% | "

                    f"CHT="
                    f"{engine.cht:5.1f} C | "

                    f"EGT="
                    f"{engine.egt:5.1f} C | "

                    f"VIB="
                    f"{engine.vib_x:.0f}/"
                    f"{engine.vib_y:.0f}/"
                    f"{engine.vib_z:.0f}"
                )


                last_telemetry_print = now


            # ================================================================
            # 6. SMALL SLEEP
            # ================================================================

            time.sleep(
                0.001
            )


    except KeyboardInterrupt:

        print()

        print(
            "Stopping simulator..."
        )


    finally:

        display.close()

        print(
            f"{PORT} closed."
        )


# =============================================================================
# PROGRAM ENTRY
# =============================================================================

if __name__ == "__main__":

    main()