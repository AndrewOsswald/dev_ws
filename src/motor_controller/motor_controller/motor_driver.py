import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Define GPIO pins for both motors
        self.left_IN1 = 15
        self.left_IN2 = 18
        self.left_ENA = 14  # PWM pin for left motor speed control

        self.right_IN1 = 23
        self.right_IN2 = 24
        self.right_ENA = 25  # PWM pin for right motor speed control

        # Set up the left and right motors and PWM speed control
        self.left_motor = Motor(forward=self.left_IN1, backward=self.left_IN2)
        self.left_pwm = PWMOutputDevice(self.left_ENA)

        self.right_motor = Motor(forward=self.right_IN1, backward=self.right_IN2)
        self.right_pwm = PWMOutputDevice(self.right_ENA)

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)  # Queue size

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Turning speed (rotation)

        # Differential drive logic
        left_speed, right_speed = self.calculate_motor_speeds(linear_velocity, angular_velocity)

        # Set motor speeds and directions
        self.apply_motor_speed(self.left_motor, self.left_pwm, left_speed)
        self.apply_motor_speed(self.right_motor, self.right_pwm, right_speed)

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

    def apply_motor_speed(self, motor, pwm, speed):
        """
        Apply the calculated speed to the motor, ensuring proper direction and speed clamping.
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