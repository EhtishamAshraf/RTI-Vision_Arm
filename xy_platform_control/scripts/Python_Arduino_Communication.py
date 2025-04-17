#! /usr/bin/env python3
"""
Controlling Arduino based XY Platform from Python:
"""
import serial, time

# Open serial connection
arduino = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=2)

time.sleep(2)  # Wait for Arduino to reset

# Read initial message from Arduino (if any)
startup_message = arduino.readline().decode().strip()
print("Arduino Startup Message:", startup_message)

# Send command to move up 500 steps
arduino.write(b"yu300\n")  # Send as bytes with a newline

# Read response from Arduino
response = arduino.readline().decode().strip()

print("Arduino Response:", response)

arduino.close()
