import smbus
import time
import math

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Addresses
multiplexer_address = 0x70  # Address of the PCA9548A multiplexer
encoder_address = 0x36      # Address of the AS5600 encoder

# Encoder and wheel characteristics
max_encoder_value = 4096         # The AS5600 provides a 12-bit angle measurement (0–4095)
update_interval = 0.001           # Adjusted time interval (in seconds) to check for velocity
wheel_diameter_in = 8            # Diameter of the wheel in inches
wheel_circumference_in = math.pi * wheel_diameter_in  # Circumference in inches

def select_channel(channel):
    """
    Selects the specified channel on the PCA9548A multiplexer.
    """
    if 0 <= channel <= 7:
        bus.write_byte(multiplexer_address, 1 << channel)
        print(f"Channel {channel} selected on PCA9548A")
    else:
        print("Invalid channel number")

def read_encoder():
    """
    Reads raw encoder data from the AS5600 encoder.
    """
    try:
        # Read 2 bytes from the encoder's angle register (0x0C and 0x0D)
        encoder_data = bus.read_i2c_block_data(encoder_address, 0x0C, 2)
        # Combine two bytes into one value
        encoder_value = encoder_data[0] << 8 | encoder_data[1]
        return encoder_value
    except IOError as e:
        print(f"Error: {e}")
        return None

def calculate_angle(value):
    """
    Converts raw encoder value to an angle in degrees (0–360).
    """
    angle = (value / max_encoder_value) * 360
    return angle

# Select channel 7
select_channel(7)

# Variables to store previous position and time for velocity calculation
previous_position = read_encoder()
previous_time = time.time()

# Moving average window size for smoothing
velocity_samples = []
average_window_size = 250

# Main loop to read encoder direction and calculate linear velocity of the wheel
while True:
    # Read the current position
    current_position = read_encoder()
    if current_position is None:
        print("Error reading encoder.")
        time.sleep(update_interval)
        continue

    # Get the current time
    current_time = time.time()
    
    # Calculate the direction in degrees (0–360 format)
    current_angle = calculate_angle(current_position)

    # Calculate velocity (change in position over time)
    delta_position = current_position - previous_position
    delta_time = current_time - previous_time

    # Handle wrap-around effect for encoder readings
    if delta_position > max_encoder_value / 2:
        delta_position -= max_encoder_value
    elif delta_position < -max_encoder_value / 2:
        delta_position += max_encoder_value

    # Calculate velocity in degrees per second
    velocity_deg_per_sec = (delta_position / delta_time) * (360 / max_encoder_value)

    # Convert angular velocity to linear velocity in inches per second
    linear_velocity_in_per_sec = (velocity_deg_per_sec / 360) * wheel_circumference_in

    # Add the current velocity to the samples for averaging
    velocity_samples.append(linear_velocity_in_per_sec)
    if len(velocity_samples) > average_window_size:
        velocity_samples.pop(0)

    # Calculate the average velocity over the sample window
    average_velocity = ( sum(velocity_samples) / len(velocity_samples) ) / 20
    average_velocity_mph = average_velocity * 0.0568182


    # Print the current angle and averaged linear velocity of the wheel
    print(f"Encoder Direction: {current_angle:.2f}°")
    print(f"Averaged Wheel Linear Velocity: {average_velocity_mph:.2f} mph")

    # Update previous position and time for next loop
    previous_position = current_position
    previous_time = current_time

    time.sleep(update_interval)
