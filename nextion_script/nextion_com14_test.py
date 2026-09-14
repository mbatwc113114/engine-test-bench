import serial
import time
import threading

# ============================================================
# NEXTION TEST SCRIPT - MATCHING YOUR NEXTION EDITOR SETTINGS
# ============================================================
# Nextion Editor screenshot:
#   COM Port : COM14
#   Baud     : 9600
#   Model    : NX8048P050_011C(TP)
#
# PC -> COM14 -> ESP32 -> GPIO16/17 -> Nextion
# ============================================================

PORT = "COM14"
BAUD = 9600

try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        write_timeout=1
    )
    print(f"CONNECTED: {PORT} @ {BAUD} baud")
except serial.SerialException as e:
    print(f"ERROR: Cannot open {PORT}")
    print(e)
    raise SystemExit(1)


def nextion_send(command):
    """
    Send an ASCII Nextion command followed by:
    FF FF FF
    """
    packet = command.encode("ascii") + b"\xFF\xFF\xFF"
    ser.write(packet)
    ser.flush()
    print(f"TX: {command}")


def receiver():
    """
    Continuously read Nextion responses.
    """
    while ser.is_open:
        try:
            data = ser.read(256)

            if data:
                print("RX HEX  :", data.hex(" "))

                # Decode readable bytes for debugging
                text = data.decode("ascii", errors="replace")
                print("RX ASCII:", repr(text))

        except serial.SerialException as e:
            print("RX ERROR:", e)
            break


# Start RX thread
rx_thread = threading.Thread(target=receiver, daemon=True)
rx_thread.start()

# Give serial connection time to settle
time.sleep(1)

print()
print("============================================================")
print(" NEXTION COM14 TEST")
print("============================================================")
print("Baud: 9600")
print()
print("Commands:")
print("  1  -> set throttle slider to 0")
print("  2  -> set throttle slider to 25")
print("  3  -> set throttle slider to 50")
print("  4  -> set throttle slider to 75")
print("  5  -> set throttle slider to 100")
print("  r  -> read throttle slider value")
print("  p  -> page 0")
print("  s  -> request current page (sendme)")
print("  q  -> quit")
print()
print("Direct commands are also supported.")
print('Example: throtleSlider.val=50')
print('Example: throtle.txt="50"')
print("============================================================")
print()

try:
    while True:

        command = input("> ").strip()

        if not command:
            continue

        # ----------------------------------------------------
        # Quit
        # ----------------------------------------------------
        if command.lower() == "q":
            break

        # ----------------------------------------------------
        # Slider tests
        # ----------------------------------------------------
        elif command == "1":
            nextion_send("throtleSlider.val=0")

        elif command == "2":
            nextion_send("throtleSlider.val=25")

        elif command == "3":
            nextion_send("throtleSlider.val=50")

        elif command == "4":
            nextion_send("throtleSlider.val=75")

        elif command == "5":
            nextion_send("throtleSlider.val=100")

        # ----------------------------------------------------
        # Read slider value
        # ----------------------------------------------------
        elif command.lower() == "r":
            nextion_send("get throtleSlider.val")

        # ----------------------------------------------------
        # Page 0
        # ----------------------------------------------------
        elif command.lower() == "p":
            nextion_send("page 0")

        # ----------------------------------------------------
        # Get current page
        # ----------------------------------------------------
        elif command.lower() == "s":
            nextion_send("sendme")

        # ----------------------------------------------------
        # Direct Nextion command
        # ----------------------------------------------------
        else:
            nextion_send(command)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    if ser.is_open:
        ser.close()

    print("COM14 closed.")
