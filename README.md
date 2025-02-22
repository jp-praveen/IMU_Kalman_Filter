# IMU - Kalman Filter - Sensor Fusion - MPU9250

This project implements a Kalman Filter for sensor fusion on an MPU9250 IMU to estimate roll, pitch, and yaw angles. It combines data from the accelerometer, gyroscope, and magnetometer to provide orientation estimates in real-time.

## Hardware Requirements
- MPU9250 IMU: Combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer.
- Arduino: Compatible with Arduino boards (e.g., Arduino Uno, Nano).
- Wiring: Connect the MPU9250 to the Arduino using I2C.

## Software Requirements
- Arduino IDE: To upload and run the code.
- MPU9250 Library: Install the MPU9250 library for Arduino.

## Code Overview
The code consists of the following key components:
- Kalman Filter: Implements a 6-state (roll, roll rate, pitch, pitch rate, yaw, yaw rate) Kalman filter for orientation estimation.
- In the code, the state vector is represented by X_BEST_ESTIMATE.
- Bias Calibration: Calibrates the accelerometer and gyroscope biases during setup.
- Sensor Fusion: Combines accelerometer, gyroscope, and magnetometer data to estimate orientation.

