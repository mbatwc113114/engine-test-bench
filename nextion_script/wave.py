import serial
import time

ser = serial.Serial("COM14", 9600, timeout=1)

def nx(cmd):
    ser.write(cmd.encode("ascii") + b"\xff\xff\xff")
    ser.flush()
    print(cmd)

# Clear waveform
nx("cle 2,0")

time.sleep(1)

# Test values
nx("add 2,0,20")
nx("add 2,0,50")
nx("add 2,0,100")
nx("add 2,0,150")
nx("add 2,0,200")
nx("add 2,0,250")

time.sleep(2)

ser.close()