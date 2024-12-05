from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice
from std_msgs.msg import String
from time import time
import rclpy
from rclpy.node import Node

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

        # Additional output pin for deceleration/stopped status (negative logic)
        self.deceleration_indicator_pin = 21
        self.deceleration_indicator = OutputDevice(self.deceleration_indicator_pin)

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

        # Define ramp rates for acceleration and deceleration
        self.accel_rate_low = 0.01   # Under 10% acceleration
        self.accel_rate_mid = 0.02   # Under 25% acceleration
        self.accel_rate_high = 0.05   # Above 25% acceleration

        self.decel_rate_low = 0.01   # Under 10% deceleration
        self.decel_rate_mid = 0.02   # Under 25% deceleration
        self.decel_rate_high = 0.05  # Above 25% deceleration

        # Timer for stopping motors on timeout
        self.last_cmd_time = time()
        self.timeout_duration = 0.5  # Stop if no new cmd_vel received in 0.5 seconds

        # E-stop status
        self.estop_status = "safe"  # Default to safe

        # Maximum speed as a percentage (0 to 100)
        self.max_speed_percentage = 75  # Limit speed to 75% of maximum

        # Create subscribers
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.estop_subscription = self.create_subscription(
            String,
            '/estop',
            self.estop_callback,
            10)

        # Timer to continuously update motor speeds for smoothing
        self.timer = self.create_timer(0.1, self.update_motor_speeds)

    def cmd_vel_callback(self, msg):
        if self.estop_status == "safe":
            # Record the time of the latest command
            self.last_cmd_time = time()

            # Calculate the target speeds for each side based on linear and angular velocities
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

            self.target_left_speed, self.target_right_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

    def estop_callback(self, msg):
        # Update e-stop status based on the message
        self.estop_status = msg.data
        self.get_logger().info(f"E-Stop status updated: {self.estop_status}")

        if self.estop_status == "unsafe":
            # Set all speeds to 0 immediately if unsafe
            self.target_left_speed = 0
            self.target_right_speed = 0
            self.current_left_speed = 0
            self.current_right_speed = 0
            self.stop_motors()

    def calculate_motor_speeds(self, linear_velocity, angular_velocity):
        max_linear_speed = 1.5
        left_speed = linear_velocity - angular_velocity
        right_speed = linear_velocity + angular_velocity

        left_speed = max(-1, min(1, left_speed / max_linear_speed)) * 100
        right_speed = max(-1, min(1, right_speed / max_linear_speed)) * 100

        # Apply maximum speed percentage
        left_speed *= self.max_speed_percentage / 100
        right_speed *= self.max_speed_percentage / 100

        return left_speed, right_speed

    def update_motor_speeds(self):
        if self.estop_status == "safe":
            # Stop motors if no command has been received recently
            if time() - self.last_cmd_time > self.timeout_duration:
                self.target_left_speed = 0
                self.target_right_speed = 0

            # Smoothly adjust current speeds toward the target speeds
            self.current_left_speed = self.ramp_toward_target(self.current_left_speed, self.target_left_speed)
            self.current_right_speed = self.ramp_toward_target(self.current_right_speed, self.target_right_speed)

            # Control deceleration indicator based on movement state
            self.update_deceleration_indicator()

            # Apply the calculated speeds to the motors
            self.set_motor_speed(self.leftfront_motor_forward, self.leftfront_motor_backward, self.leftfront_pwm, self.current_left_speed)
            self.set_motor_speed(self.leftback_motor_forward, self.leftback_motor_backward, self.leftback_pwm, self.current_left_speed)
            self.set_motor_speed(self.rightfront_motor_forward, self.rightfront_motor_backward, self.rightfront_pwm, self.current_right_speed)
            self.set_motor_speed(self.rightback_motor_forward, self.rightback_motor_backward, self.rightback_pwm, self.current_right_speed)

    def ramp_toward_target(self, current_speed, target_speed):
        abs_speed = abs(current_speed)

        if abs_speed < 10:
            accel_rate = self.accel_rate_low
            decel_rate = self.decel_rate_low
        elif abs_speed < 25:
            accel_rate = self.accel_rate_mid
            decel_rate = self.decel_rate_mid
        else:
            accel_rate = self.accel_rate_high
            decel_rate = self.decel_rate_high

        if current_speed < target_speed:
            return min(current_speed + accel_rate * 100, target_speed)
        elif current_speed > target_speed:
            return max(current_speed - decel_rate * 100, target_speed)
        else:
            return target_speed

    def update_deceleration_indicator(self):
        if self.current_left_speed > self.target_left_speed or self.current_right_speed > self.target_right_speed or \
           (self.current_left_speed == 0 and self.current_right_speed == 0):
            self.deceleration_indicator.off()
        else:
            self.deceleration_indicator.on()

    def set_motor_speed(self, forward_pin, backward_pin, pwm, speed):
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
        self.deceleration_indicator.on()
        self.get_logger().info('All motors stopped')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
