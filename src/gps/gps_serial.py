import serial
import sys

ser = serial.Serial("/dev/ttyUSB0", 4800)

try:
    while True:
        try:
            # byte to string
            data = ser.readline().decode("utf-8")
            print(data)
        except Exception as e:
            print(f"An error occured: {e}")
            sys.exit(0)
except KeyboardInterrupt:
    print("\nKeyboard interrupt detected. Closing serial port!")
    ser.close()
    sys.exit(0)