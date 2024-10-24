import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import OutputDevice, PWMOutputDevice

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Define GPIO pins for front and rear motors on both sides
        # Front left motor
        self.leftfront_IN1 = 14  # GPIO pin for front left motor forward
        self.leftfront_IN2 = 15  # GPIO pin for front left motor backward
        self.leftfront_ENA = 12  # PWM pin for front left motor speed control

        # Front right motor
        self.rightfront_IN1 = 23  # GPIO pin for front right motor forward
        self.rightfront_IN2 = 24  # GPIO pin for front right motor backward
        self.rightfront_ENA = 25  # PWM pin for front right motor speed control

        # Back left motor
        self.leftback_IN1 = 27  # GPIO pin for back left motor forward
        self.leftback_IN2 = 22  # GPIO pin for back left motor backward
        self.leftback_ENA = 17  # PWM pin for back left motor speed control

        # Back right motor
        self.rightback_IN1 = 10  # GPIO pin for back right motor forward
        self.rightback_IN2 = 9  # GPIO pin for back right motor backward
        self.rightback_ENA = 11  # PWM pin for back right motor speed control

        # Set up the front and back motors on both sides with PWM speed control
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

        # Stop motors if both velocities are zero
        if linear_velocity == 0 and angular_velocity == 0:
            self.stop_motors()
        else:
            # Set motor speeds and directions for left motors
            if left_speed >= 0:
                self.set_motor(self.leftfront_motor_forward, self.leftfront_motor_backward, self.leftfront_pwm, 'forward', left_speed)
                self.set_motor(self.leftback_motor_forward, self.leftback_motor_backward, self.leftback_pwm, 'forward', left_speed)
            else:
                self.set_motor(self.leftfront_motor_forward, self.leftfront_motor_backward, self.leftfront_pwm, 'backward', abs(left_speed))
                self.set_motor(self.leftback_motor_forward, self.leftback_motor_backward, self.leftback_pwm, 'backward', abs(left_speed))

            # Set motor speeds and directions for right motors
            if right_speed >= 0:
                self.set_motor(self.rightfront_motor_forward, self.rightfront_motor_backward, self.rightfront_pwm, 'forward', right_speed)
                self.set_motor(self.rightback_motor_forward, self.rightback_motor_backward, self.rightback_pwm, 'forward', right_speed)
            else:
                self.set_motor(self.rightfront_motor_forward, self.rightfront_motor_backward, self.rightfront_pwm, 'backward', abs(right_speed))
                self.set_motor(self.rightback_motor_forward, self.rightback_motor_backward, self.rightback_pwm, 'backward', abs(right_speed))

    def calculate_motor_speeds(self, linear_velocity, angular_velocity):
        """
        Calculate the speeds for the left and right motors based on linear and angular velocities.
        """

        # Adjust for turning. Angular velocity adjusts the speed difference between the wheels.
        left_speed = linear_velocity - angular_velocity
        right_speed = linear_velocity + angular_velocity

        # Normalize the speeds based on maximum possible linear and angular values
        max_linear_speed = 1.5  # Assuming 1.5 is 100% forward/backward speed
        max_angular_speed = 1.0  # Full turning speed
    
        # Scale motor speeds to ensure values are within the valid range (-1 to 1)
        left_speed = max(-1, min(1, left_speed / max_linear_speed))
        right_speed = max(-1, min(1, right_speed / max_linear_speed))

        return left_speed * 100, right_speed * 100  # Return as percentage

    def set_motor(self, forward_pin, backward_pin, pwm, direction, speed):
        """
        Set motor direction and speed.
        :param forward_pin: GPIO pin for forward direction
        :param backward_pin: GPIO pin for backward direction
        :param pwm: PWM instance for the motor
        :param direction: 'forward' or 'backward'
        :param speed: Speed of the motor (0-100%)
        """
        # Clamp speed between 0 and 100
        speed = max(0, min(100, speed))

        if direction == 'forward':
            forward_pin.on()
            backward_pin.off()
        elif direction == 'backward':
            forward_pin.off()
            backward_pin.on()

        # Set speed (convert to range 0.0 to 1.0 for gpiozero PWM)
        pwm.value = speed / 100.0
        self.get_logger().info(f'Motor {direction} at {speed}% speed')

    def stop_motors(self):
        """Stop all motors."""
        # Stop left front motors
        self.leftfront_motor_forward.off()
        self.leftfront_motor_backward.off()
        self.leftfront_pwm.value = 0

        # Stop right front motors
        self.rightfront_motor_forward.off()
        self.rightfront_motor_backward.off()
        self.rightfront_pwm.value = 0

        # Stop left back motors
        self.leftback_motor_forward.off()
        self.leftback_motor_backward.off()
        self.leftback_pwm.value = 0

        # Stop right back motors
        self.rightback_motor_forward.off()
        self.rightback_motor_backward.off()
        self.rightback_pwm.value = 0

        self.get_logger().info('All motors stopped')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()