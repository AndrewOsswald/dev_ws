import smbus
import time
import math

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Addresses
multiplexer_address = 0x70  # Address of the PCA9548A multiplexer
encoder_address = 0x36      # Address of the AS5600 encoder

# Encoder and wheel characteristics
max_encoder_value = 4096          # The AS5600 provides a 12-bit angle measurement (0–4095)
update_interval = 0.01            # Increased time interval to help stabilize readings
wheel_diameter_in = 8             # Diameter of the wheel in inches
wheel_circumference_in = math.pi * wheel_diameter_in  # Circumference in inches

# Define channels for each wheel
channels = {
    "front_left": 0,
    "front_right": 1,
    "back_left": 2,
    "back_right": 3
}

# Moving average window size for smoothing
average_window_size = 250
velocity_samples = {wheel: [] for wheel in channels}
stable_threshold = 2  # Threshold for changes to ignore as noise when stationary

# Functions
def select_channel(channel):
    try:
        if 0 <= channel <= 7:
            bus.write_byte(multiplexer_address, 1 << channel)
            time.sleep(0.001)  # Small delay to allow for stable channel selection
        else:
            print("Invalid channel number")
    except Exception as e:
        #print(f"Error selecting channel {channel} on multiplexer: {e}")
        return None

def read_encoder():
    try:
        encoder_data = bus.read_i2c_block_data(encoder_address, 0x0C, 2)
        encoder_value = encoder_data[0] << 8 | encoder_data[1]
        return encoder_value
    except IOError as e:
        #print(f"Error reading from encoder: {e}")
        return None

def calculate_angle(value):
    return (value / max_encoder_value) * 360

# Main loop to read encoder direction and calculate linear velocity for each wheel
previous_positions = {wheel: read_encoder() or 0 for wheel in channels}
previous_times = {wheel: time.time() for wheel in channels}

while True:
    output = []  # List to store output strings for each wheel

    for wheel, channel in channels.items():
        # Select the appropriate channel on the multiplexer
        select_channel(channel)

        # Read the current position
        current_position = read_encoder()
        if current_position is None:
            output.append(f"{wheel.capitalize()}: Error reading encoder. Skipping this cycle.")
            output.append(f" ")
            time.sleep(update_interval)
            continue

        # Get the current time
        current_time = time.time()

        # Calculate the direction in degrees (0–360 format)
        current_angle = calculate_angle(current_position)

        # Calculate velocity (change in position over time)
        try:
            delta_position = current_position - previous_positions[wheel]
            delta_time = current_time - previous_times[wheel]

            # Handle wrap-around effect for encoder readings
            if delta_position > max_encoder_value / 2:
                delta_position -= max_encoder_value
            elif delta_position < -max_encoder_value / 2:
                delta_position += max_encoder_value

            # Ignore tiny changes when stationary
            if abs(delta_position) < stable_threshold:
                delta_position = 0

            # Calculate velocity in degrees per second if delta_time > 0
            velocity_deg_per_sec = (delta_position / delta_time) * (360 / max_encoder_value) if delta_time > 0 else 0

            # Convert angular velocity to linear velocity in inches per second
            linear_velocity_in_per_sec = (velocity_deg_per_sec / 360) * wheel_circumference_in

            # Add the current velocity to the samples for averaging
            velocity_samples[wheel].append(linear_velocity_in_per_sec)
            if len(velocity_samples[wheel]) > average_window_size:
                velocity_samples[wheel].pop(0)

            # Calculate the average velocity over the sample window
            average_velocity_in_per_sec = sum(velocity_samples[wheel]) / len(velocity_samples[wheel])
            average_velocity_mph = average_velocity_in_per_sec * 0.0568182

            # Store formatted output for the wheel
            output.append(f"{wheel.capitalize()} Encoder Direction: {current_angle:.2f}°")
            output.append(f"{wheel.capitalize()} Averaged Wheel Linear Velocity: {average_velocity_mph:.2f} mph")

            # Update previous position and time for next loop
            previous_positions[wheel] = current_position
            previous_times[wheel] = current_time

        except ZeroDivisionError:
            output.append(f"{wheel.capitalize()}: Time delta was too small for velocity calculation. Skipping this cycle.")
            output.append(f" ")
        except Exception as e:
            output.append(f"{wheel.capitalize()}: Error calculating velocity: {e}")
            output.append(f" ")

    # Print all output for this cycle at once
    print("\n".join(output))
    print("-" * 40)

    time.sleep(update_interval)
