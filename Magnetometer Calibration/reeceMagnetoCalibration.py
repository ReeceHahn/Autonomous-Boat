import serial
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Configure the serial connection
SERIAL_PORT = "COM6"
BAUD_RATE = 115200
OUTPUT_FILE = "magnetometer_data.txt"

AXIS_LIMITS = (-0.6, 0.6)  # Adjust the range for the axes

def read_serial_data(serial_connection):
    try:
        line = serial_connection.readline().decode('utf-8').strip()
        x, y, z = map(float, line.split(","))
        return x, y, z
    except Exception as e:
        print("Error reading data:", e)
        return None

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("Magnetometer Data Visualization")

x_data, y_data, z_data = [], [], []

# Open file for writing
data_file = open(OUTPUT_FILE, "w")

try:
    while True:
        data = read_serial_data(ser)
        if data:
            x, y, z = data
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)
            
            # Write data to file
            data_file.write(f"{x}\t{y}\t{z}\n")
            
            ax.clear()
            ax.set_title("Magnetometer Data Visualization")
            ax.set_xlabel("X Axis")
            ax.set_ylabel("Y Axis")
            ax.set_zlabel("Z Axis")

            # Set fixed axis limits
            ax.set_xlim(AXIS_LIMITS)
            ax.set_ylim(AXIS_LIMITS)
            ax.set_zlim(AXIS_LIMITS)

            # Ensure equal scaling for all axes
            ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio for x, y, z axes

            ax.scatter(x_data, y_data, z_data, c='r', marker='o')

            # Update the plot without blocking the event loop
            plt.pause(0.05)

except KeyboardInterrupt:
    print("⛔ Stopped by user")
    ser.close()
    data_file.close()
    plt.show()