import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Define GPIO pins for both motors
        self.left_IN1 = 15
        self.left_IN2 = 18
        self.left_ENA = 14  # PWM pin for left motor speed control

        self.right_IN1 = 8
        self.right_IN2 = 7
        self.right_ENA = 25  # PWM pin for right motor speed control

        # Set up the left and right motors and PWM speed control
        self.left_motor = Motor(forward=self.left_IN1, backward=self.left_IN2)
        self.left_pwm = PWMOutputDevice(self.left_ENA)

        self.right_motor = Motor(forward=self.right_IN1, backward=self.right_IN2)
        self.right_pwm = PWMOutputDevice(self.right_ENA)

        # Smoothing parameters
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.ramp_rate = 5  # The rate of change per update (adjust as needed)

        # Time tracking for /cmd_vel message timeout
        self.last_cmd_vel_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.timeout_duration = 0.5  # Timeout after 0.5 seconds of no new message

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)  # Queue size

        # Create a timer to check for timeout and ramp down motors if needed
        self.create_timer(0.1, self.check_timeout)  # Check every 100 ms

    def cmd_vel_callback(self, msg):
        # Update the last message received time
        self.last_cmd_vel_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Extract linear and angular velocities
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Turning speed (rotation)

        # Differential drive logic
        target_left_speed, target_right_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

        # Smoothly adjust motor speeds towards the target speeds
        self.current_left_speed = self.smooth_speed(self.current_left_speed, target_left_speed)
        self.current_right_speed = self.smooth_speed(self.current_right_speed, target_right_speed)

        # Apply the smoothed motor speeds
        self.apply_motor_speed(self.left_motor, self.left_pwm, self.current_left_speed)
        self.apply_motor_speed(self.right_motor, self.right_pwm, self.current_right_speed)

    def calculate_motor_speeds(self, linear_velocity, angular_velocity):
        """
        Calculate the speeds for the left and right motors based on linear and angular velocities.
        """
        # Calculate differential speeds for each motor
        left_speed = linear_velocity - angular_velocity
        right_speed = linear_velocity + angular_velocity

        # Clamp the speeds to the range -100 to 100
        left_speed = max(-100, min(100, left_speed * 100))  # Convert to percentage
        right_speed = max(-100, min(100, right_speed * 100))  # Convert to percentage

        return left_speed, right_speed

    def smooth_speed(self, current_speed, target_speed):
        """
        Gradually adjust the current speed towards the target speed for smoother transitions.
        The ramp rate is 5 times slower for speeds below 25%.
        :param current_speed: The current speed of the motor.
        :param target_speed: The desired target speed of the motor.
        :return: The new smoothed speed.
        """
        # Set a slower ramp rate for speeds below 25%
        if abs(current_speed) < 25 and abs(target_speed) < 25:
            adjusted_ramp_rate = self.ramp_rate / 5  # 5x slower ramp rate
        else:
            adjusted_ramp_rate = self.ramp_rate

        if current_speed < target_speed:
            return min(current_speed + adjusted_ramp_rate, target_speed)
        elif current_speed > target_speed:
            return max(current_speed - adjusted_ramp_rate, target_speed)
        return current_speed

    def apply_motor_speed(self, motor, pwm, speed):
        """
        Apply the smoothed speed to the motor, ensuring proper direction and speed clamping.
        :param motor: Motor instance (left or right motor)
        :param pwm: PWM instance for the motor
        :param speed: Speed of the motor (-100 to 100)
        """
        # If speed is positive, go forward; if negative, go backward
        if speed > 0:
            motor.forward()
            pwm.value = speed / 100.0  # Set PWM value
        elif speed < 0:
            motor.backward()
            pwm.value = abs(speed) / 100.0  # Set PWM value (positive for reverse)
        else:
            motor.stop()
            pwm.value = 0  # Stop the motor

        self.get_logger().info(f'Motor running at {speed}% speed')

    def check_timeout(self):
        """
        Check if the time since the last /cmd_vel message exceeds the timeout duration.
        If it does, ramp down the motor speeds to zero.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_cmd_vel_time > self.timeout_duration:
            # No new /cmd_vel message received, assume the target speed is zero
            self.current_left_speed = self.smooth_speed(self.current_left_speed, 0)
            self.current_right_speed = self.smooth_speed(self.current_right_speed, 0)

            # Apply the smoothed motor speeds to gradually stop the motors
            self.apply_motor_speed(self.left_motor, self.left_pwm, self.current_left_speed)
            self.apply_motor_speed(self.right_motor, self.right_pwm, self.current_right_speed)

    def stop_motors(self):
        """Stop both motors."""
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_pwm.value = 0
        self.right_pwm.value = 0
        self.get_logger().info('Both motors stopped')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
