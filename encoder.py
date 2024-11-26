import smbus
import time
from collections import deque

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Encoder and multiplexer addresses
multiplexer_address = 0x70  # Address of the PCA9548A multiplexer
encoder_address = 0x36      # Address of the AS5600 encoder

# Encoder characteristics
max_encoder_value = 4096    # The AS5600 provides a 12-bit angle measurement (0-4095)

# Define channels for each wheel
channels = {
    "front_left": 0,
    "front_right": 1,
    "back_left": 2,
    "back_right": 3
}

# RPM sampling window size (how many previous RPMs to average)
sampling_window_size = 10
rpm_samples = {wheel: deque(maxlen=sampling_window_size) for wheel in channels}

# Store previous encoder positions and times for each wheel
previous_positions = {wheel: 0 for wheel in channels}
previous_times = {wheel: None for wheel in channels}

# Function to select the appropriate channel on the multiplexer
def select_channel(channel):
    bus.write_byte(multiplexer_address, 1 << channel)

# Function to read the encoder value
def read_encoder():
    encoder_data = bus.read_i2c_block_data(encoder_address, 0x0C, 2)
    return encoder_data[0] << 8 | encoder_data[1]

# Function to calculate RPM
def calculate_rpm(delta_position, delta_time):
    if delta_time <= 0:
        return 0
    return (delta_position / max_encoder_value) * (60 / delta_time)

# Function to calculate the average RPM from the sampling window
def get_average_rpm(wheel):
    if len(rpm_samples[wheel]) == 0:
        return 0
    return sum(rpm_samples[wheel]) / len(rpm_samples[wheel])

# Main loop to calculate RPM for each wheel
while True:
    output = []  # List to store output for all wheels

    for wheel, channel in channels.items():
        select_channel(channel)

        # Read the current encoder position for the wheel
        current_position = read_encoder()

        # Get the current time
        current_time = time.time()

        # Calculate the delta position and delta time
        if previous_times[wheel] is not None:
            delta_position = current_position - previous_positions[wheel]

            # Handle wrap-around effect for encoder readings
            if delta_position > max_encoder_value / 2:
                delta_position -= max_encoder_value
            elif delta_position < -max_encoder_value / 2:
                delta_position += max_encoder_value

            # Calculate the time difference
            delta_time = current_time - previous_times[wheel]

            # Calculate RPM
            rpm = calculate_rpm(delta_position, delta_time)

            # Add the current RPM to the samples for averaging
            rpm_samples[wheel].append(rpm)

            # Get the average RPM from the sampling window
            average_rpm = get_average_rpm(wheel)

            # Store the output for this wheel
            output.append(f"{wheel.capitalize()} - Average RPM: {average_rpm:.2f}")

        # Update previous position and time for the next loop
        previous_positions[wheel] = current_position
        previous_times[wheel] = current_time

    # Print all output for all wheels
    print("\n".join(output))
    print("-" * 40)

    # Sleep as little as possible to allow for faster sampling
    time.sleep(0.0001)  # Minimized sleep to reduce delay between sampling
