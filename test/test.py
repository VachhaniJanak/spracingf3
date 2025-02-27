# import serial
# import time

# # Replace '/dev/ttyUSB0' with your port name, and adjust the baud rate as needed
# serial_port = '/dev/ttyUSB0'  # or 'COM3' on Windows
# baud_rate = 9600              # set your baud rate

# # Open the serial port
# ser = serial.Serial(serial_port, baud_rate, timeout=1)

# # Allow some time for the serial connection to initialize
# time.sleep(2)

# try:
#     while True:
#         if ser.in_waiting > 0:           # Check if there is data in the buffer
#             line = ser.readline().decode('utf-8').strip()  # Read a line
#             print(line)     # Print the received data
# except KeyboardInterrupt:
#     print("Stopping data read.")
# finally:
#     ser.close()                          # Always close the serial port


import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
# import tmp102


# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Initialize communication with TMP102
# tmp102.init()

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    temp_c = round(np.random.rand(), 2)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(temp_c)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[:]

    # Draw x and y lists
    ax.clear()
    ax.plot(ys)

    # Format plot
    plt.grid()
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=100)
plt.show()
