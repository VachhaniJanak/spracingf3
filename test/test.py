import serial
import time

# Replace '/dev/ttyUSB0' with your port name, and adjust the baud rate as needed
serial_port = '/dev/ttyUSB0'  # or 'COM3' on Windows
baud_rate = 9600              # set your baud rate

# Open the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Allow some time for the serial connection to initialize
time.sleep(2)

try:
    while True:
        if ser.in_waiting > 0:           # Check if there is data in the buffer
            line = ser.readline().decode('utf-8').strip()  # Read a line
            print(line)     # Print the received data
except KeyboardInterrupt:
    print("Stopping data read.")
finally:
    ser.close()                          # Always close the serial port
