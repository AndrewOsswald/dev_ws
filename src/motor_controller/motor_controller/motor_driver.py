import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice
from time import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Define GPIO pins for front and rear motors on both sides
        # Front left motor
        self.leftfront_IN1 = 17
        self.leftfront_IN2 = 27
        self.leftfront_ENA = 13

        # Front right motor
        self.rightfront_IN1 = 14
        self.rightfront_IN2 = 15
        self.rightfront_ENA = 12

        # Back left motor
        self.leftback_IN1 = 22
        self.leftback_IN2 = 23
        self.leftback_ENA = 19

        # Back right motor
        self.rightback_IN1 = 5
        self.rightback_IN2 = 6
        self.rightback_ENA = 18

        # Set up the motor control and PWM instances
        self.leftfront_motor_forward = OutputDevice(self.leftfront_IN1)
        self.leftfront_motor_backward = OutputDevice(self.leftfront_IN2)
        self.leftfront_pwm = PWMOutputDevice(self.leftfront_ENA)

        self.rightfront_motor_forward = OutputDevice(self.rightfront_IN1)
        self.rightfront_motor_backward = OutputDevice(self.rightfront_IN2)
        self.rightfront_pwm = PWMOutputDevice(self.rightfront_ENA)

        self.leftback_motor_forward = OutputDevice(self.leftback_IN1)
        self.leftback_motor_backward = OutputDevice(self.leftback_IN2)
        self.leftback_pwm = PWMOutputDevice(self.leftback_ENA)

        self.rightback_motor_forward = OutputDevice(self.rightback_IN1)
        self.rightback_motor_backward = OutputDevice(self.rightback_IN2)
        self.rightback_pwm = PWMOutputDevice(self.rightback_ENA)

        # Motor target and current speeds
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.current_left_speed = 0
        self.current_right_speed = 0

        # Smoothing parameters
        self.ramp_rate_high = 0.10  # High speed ramp rate
        self.ramp_rate_low = 0.02  # Low speed ramp rate for speeds below 25%

        # Timer for stopping motors on timeout
        self.last_cmd_time = time()
        self.timeout_duration = 0.5  # Stop if no new cmd_vel received in 0.5 seconds

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Timer to continuously update motor speeds for smoothing
        self.timer = self.create_timer(0.1, self.update_motor_speeds)

    def cmd_vel_callback(self, msg):
        # Record the time of the latest command
        self.last_cmd_time = time()

        # Calculate the target speeds for each side based on linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        self.target_left_speed, self.target_right_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

    def calculate_motor_speeds(self, linear_velocity, angular_velocity):
        max_linear_speed = 1.5
        left_speed = linear_velocity - angular_velocity
        right_speed = linear_velocity + angular_velocity

        left_speed = max(-1, min(1, left_speed / max_linear_speed)) * 100
        right_speed = max(-1, min(1, right_speed / max_linear_speed)) * 100

        return left_speed, right_speed

    def update_motor_speeds(self):
        # Stop motors if no command has been received recently
        if time() - self.last_cmd_time > self.timeout_duration:
            self.target_left_speed = 0
            self.target_right_speed = 0

        # Smoothly adjust current speeds toward the target speeds
        self.current_left_speed = self.ramp_toward_target(self.current_left_speed, self.target_left_speed)
        self.current_right_speed = self.ramp_toward_target(self.current_right_speed, self.target_right_speed)

        # Apply the calculated speeds to the motors
        self.set_motor_speed(self.leftfront_motor_forward, self.leftfront_motor_backward, self.leftfront_pwm, self.current_left_speed)
        self.set_motor_speed(self.leftback_motor_forward, self.leftback_motor_backward, self.leftback_pwm, self.current_left_speed)
        self.set_motor_speed(self.rightfront_motor_forward, self.rightfront_motor_backward, self.rightfront_pwm, self.current_right_speed)
        self.set_motor_speed(self.rightback_motor_forward, self.rightback_motor_backward, self.rightback_pwm, self.current_right_speed)

    def ramp_toward_target(self, current_speed, target_speed):
        # Choose ramp rate based on current speed level
        ramp_rate = self.ramp_rate_low if abs(current_speed) < 25 else self.ramp_rate_high

        # Ramp up or down towards the target speed
        if current_speed < target_speed:
            return min(current_speed + ramp_rate * 100, target_speed)
        elif current_speed > target_speed:
            return max(current_speed - ramp_rate * 100, target_speed)
        else:
            return target_speed

    def set_motor_speed(self, forward_pin, backward_pin, pwm, speed):
        # Ensure speed is within 0-100 for PWM
        abs_speed = min(100, abs(speed))
        direction = 'forward' if speed >= 0 else 'backward'

        if direction == 'forward':
            forward_pin.on()
            backward_pin.off()
        else:
            forward_pin.off()
            backward_pin.on()

        pwm.value = abs_speed / 100.0
        self.get_logger().info(f'Motor {direction} at {abs_speed}% speed')

    def stop_motors(self):
        self.set_motor_speed(self.leftfront_motor_forward, self.leftfront_motor_backward, self.leftfront_pwm, 0)
        self.set_motor_speed(self.leftback_motor_forward, self.leftback_motor_backward, self.leftback_pwm, 0)
        self.set_motor_speed(self.rightfront_motor_forward, self.rightfront_motor_backward, self.rightfront_pwm, 0)
        self.set_motor_speed(self.rightback_motor_forward, self.rightback_motor_backward, self.rightback_pwm, 0)
        self.get_logger().info('All motors stopped')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
