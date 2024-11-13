import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import PWMLED

class Salter(Node):
    def __init__(self):
        super().__init__('salter')

        # Set up PWM pin 9 for controlling the salter
        self.salter_pwm = PWMLED(9)

        # Define PWM duty cycle settings for each speed level
        self.speed_levels = [0.0, 0.3, 0.6, 1.0]  # Off, slow, medium, fast
        self.current_speed_index = 0  # Start at 'off'

        # Track previous button states to detect single presses
        self.prev_up_button_state = 0.0
        self.prev_down_button_state = 0.0

        # Update PWM to initial speed
        self.update_pwm()

        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.get_logger().info('Salter node has been started, listening to /joy topic...')

    def joy_callback(self, msg):
        # Check the 8th element in the axes array (index 7) for up and down button states
        if len(msg.axes) > 7:
            up_button_state = msg.axes[7] == 1.0
            down_button_state = msg.axes[7] == -1.0

            # Increase speed on a single press of the "up" button
            if up_button_state and not self.prev_up_button_state:
                self.increase_speed()

            # Decrease speed on a single press of the "down" button
            if down_button_state and not self.prev_down_button_state:
                self.decrease_speed()

            # Update previous button states
            self.prev_up_button_state = up_button_state
            self.prev_down_button_state = down_button_state

    def increase_speed(self):
        # Only increase speed if we haven't reached the max speed
        if self.current_speed_index < len(self.speed_levels) - 1:
            self.current_speed_index += 1
            self.update_pwm()
            self.get_logger().info(f'Salter increased to {self.get_speed_label()} speed.')

    def decrease_speed(self):
        # Only decrease speed if we're above the minimum speed (off)
        if self.current_speed_index > 0:
            self.current_speed_index -= 1
            self.update_pwm()
            self.get_logger().info(f'Salter decreased to {self.get_speed_label()} speed.')

    def update_pwm(self):
        # Update the PWM duty cycle based on the current speed index
        self.salter_pwm.value = self.speed_levels[self.current_speed_index]

    def get_speed_label(self):
        # Helper function to get the current speed label
        labels = ['off', 'slow', 'medium', 'fast']
        return labels[self.current_speed_index]

    def stop_salter(self):
        # Ensure the salter is turned off when stopping
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

