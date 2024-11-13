import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import PWMLED

class Salter(Node):
    def __init__(self):
        super().__init__('salter')

        # Set up PWM pin 9 for controlling the salter
        self.salter_pwm = PWMLED(9)

        # PWM duty cycle settings for each speed
        self.speeds = {
            'off': 0.0,
            'slow': 0.1,
            'medium': 0.2,
            'fast': 0.3
        }

        # Initialize speed to 'off'
        self.current_speed = 'off'
        self.update_pwm(self.current_speed)

        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.get_logger().info('Salter node has been started, listening to /joy topic...')

    def joy_callback(self, msg):
        # Check the 8th element in the axes array (index 7)
        if len(msg.axes) > 7:
            axis_value = msg.axes[7]

            # Set speed based on the axis value
            if axis_value == 1.0:
                self.set_speed('fast')
            elif axis_value == -1.0:
                self.set_speed('slow')
            else:
                self.set_speed('off')

    def set_speed(self, speed):
        # Only update if the speed has changed
        if speed != self.current_speed:
            self.current_speed = speed
            self.update_pwm(speed)
            self.get_logger().info(f'Salter set to {speed} speed.')

    def update_pwm(self, speed):
        # Update the PWM duty cycle based on speed setting
        self.salter_pwm.value = self.speeds[speed]

    def stop_salter(self):
        # Ensure salter is turned off when stopping
        self.salter_pwm.off()
        self.get_logger().info('Salter stopped.')

def main(args=None):
    rclpy.init(args=args)
    salter = Salter()
    rclpy.spin(salter)
    salter.stop_salter()
    salter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
