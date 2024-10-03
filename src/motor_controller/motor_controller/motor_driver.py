import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice
from time import sleep

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Define GPIO pins for motor control
        self.IN1 = 14
        self.IN2 = 15
        self.ENA = 12  # PWM pin for speed control

        # Set up the motor and PWM speed control
        self.motor = Motor(forward=self.IN1, backward=self.IN2)
        self.pwm = PWMOutputDevice(self.ENA)

        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)  # Queue size

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Not used in this basic motor control

        # Translate linear velocity to motor direction and speed
        if linear_velocity > 0:
            self.set_motor('forward', linear_velocity * 100)  # Convert to percentage speed
        elif linear_velocity < 0:
            self.set_motor('backward', abs(linear_velocity) * 100)  # Convert to percentage speed
        else:
            self.stop_motor()  # Stop motor if no linear velocity

    def set_motor(self, direction, speed):
        # """
        # Set motor direction and speed.
        # :param direction: 'forward' or 'backward'
        # :param speed: Speed of the motor (0-100%)
        # """
        if direction == 'forward':
            self.motor.forward()
        elif direction == 'backward':
            self.motor.backward()
        else:
            raise ValueError("Direction must be 'forward' or 'backward'")

        # Set speed (convert to range 0.0 to 1.0 for gpiozero)
        self.pwm.value = speed / 100.0
        self.get_logger().info(f'Motor {direction} at {speed}% speed')

    def stop_motor(self):
        # """Stop the motor."""
        self.motor.stop()
        self.pwm.value = 0
        self.get_logger().info('Motor stopped')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()