#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

platform = "robot"  # robot or car
# Common base directory
base_dir = "yourpath\\Wheel-INS" if os.name == "nt" else "~/Wheel-INS"

# Platform-specific paths
path = os.path.join(base_dir, "output", platform, "traj.txt")
imupath = os.path.join(base_dir, "dataset", platform, "Wheel-IMU", "C1_imu.bin")

num_columns = 7
dtype = np.float64

# Read the binary file using NumPy
try:
    imu = np.fromfile(imupath, dtype=dtype)
    
    # Reshape the data into a 2D array with 7 columns
    imu = imu.reshape(-1, num_columns)

    print("Successfully read the binary file.")
    print("Shape of the loaded data:", imu.shape)
except FileNotFoundError:
    print(f"File not found at path: {imupath}")
except Exception as e:
    print(f"An error occurred: {str(e)}")

fig, axs = plt.subplots(2, 1, figsize=(8,7))
axs[0].plot(imu[:, 0] - imu[0, 0], imu[:, 1])
axs[0].plot(imu[:, 0] - imu[0, 0], imu[:, 2])
axs[0].plot(imu[:, 0] - imu[0, 0], imu[:, 3])
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Gyro (rad/s)')
axs[0].legend(['X', 'Y', 'Z'])
axs[0].set_title( 'imu raw data (' + platform + ')')
axs[1].plot(imu[:, 0] - imu[0, 0], imu[:, 4])
axs[1].plot(imu[:, 0] - imu[0, 0], imu[:, 5])
axs[1].plot(imu[:, 0] - imu[0, 0], imu[:, 6])
axs[1].set_xlabel('Time (s) ' + str(imu[0, 0]))
axs[1].set_ylabel('Acc (m/s2)')
axs[1].legend(['X', 'Y', 'Z'])

traj = np.loadtxt(path)
fig, ax = plt.subplots(figsize=(8,7))
ax.plot(traj[:, 1], traj[:, 2])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.axis('equal')
ax.set_title( 'trajectory (' + platform + ')')

plt.show()