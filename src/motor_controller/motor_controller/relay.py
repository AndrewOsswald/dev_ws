import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import OutputDevice

class RelayController(Node):
    def __init__(self):
        super().__init__('relay_controller')

        # Set up GPIO pin for the relay
        self.relay_pin = 10
        self.relay = OutputDevice(self.relay_pin)

        # Store the last button state to detect changes
        self.last_button_state = 0

        # Create a subscriber to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.get_logger().info('RelayController node has been started, listening to /joy topic...')

    def joy_callback(self, msg):
        # Check if the last button in the buttons array is pressed (index 12)
        if len(msg.buttons) >= 13:
            current_button_state = msg.buttons[12]

            # Toggle relay only when the button state changes to 1
            if current_button_state == 1 and self.last_button_state == 0:
                self.toggle_relay()

            # Update the last button state
            self.last_button_state = current_button_state

    def toggle_relay(self):
        # Toggle the relay
        if self.relay.is_active:
            self.relay.off()
            self.get_logger().info("Relay turned OFF")
        else:
            self.relay.on()
            self.get_logger().info("Relay turned ON")

    def stop_relay(self):
        # Ensure relay is turned off when stopping
        self.relay.off()
        self.get_logger().info("Relay stopped")

def main(args=None):
    rclpy.init(args=args)
    relay_controller = RelayController()
    rclpy.spin(relay_controller)
    relay_controller.stop_relay()
    relay_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
