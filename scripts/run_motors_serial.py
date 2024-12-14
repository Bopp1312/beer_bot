import time
import math
import serial

# Set up the serial connection to Arduino
# Adjust the port and baud rate as per your Arduino setup
ser = serial.Serial('/dev/ttyACM0', 115200)

# Parameters for the sinusoidal wave
frequency = 0.1  # Frequency in Hz
amplitude = 10000  # Maximum command value
offset = 0       # Offset value for the sine wave

# Function to send motor commands
def send_motor_command(left_motor, right_motor):
    # Create a command string
    command = f"{left_motor},{right_motor}\n"
    # Send the command to the Arduino
    ser.write(command.encode())

try:
    print("Sending sinusoidal commands. Press Ctrl+C to stop.")
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        # Compute sinusoidal values
        left_motor = int(offset + amplitude * math.sin(2 * math.pi * frequency * elapsed_time))
        right_motor = int(offset + amplitude * math.sin(2 * math.pi * frequency * elapsed_time + math.pi / 2))  # Phase shift for right motor

        # Send motor commands
        send_motor_command(left_motor, right_motor)

        # Print for debugging
        print(f"Left Motor: {left_motor}, Right Motor: {right_motor}")

        # Wait for the next cycle (e.g., 50 Hz)
        time.sleep(0.2)  # 20 ms
except KeyboardInterrupt:
    send_motor_command(0,0)
    print("Stopped by user.")
finally:
    ser.close()

