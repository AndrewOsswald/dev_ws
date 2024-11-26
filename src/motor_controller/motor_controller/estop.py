import rclpy
from rclpy.node import Node
from gpiozero import InputDevice, OutputDevice
from std_msgs.msg import String

class EStopMonitor(Node):
    def __init__(self):
        super().__init__('estop_monitor')

        # GPIO pin setup for e-stop and LED indicator
        self.estop_pin = 26
        self.estop = InputDevice(self.estop_pin, pull_up=False)  # Active-high logic (3.3V = safe)

        self.led_pin = 21
        self.led = OutputDevice(self.led_pin)

        # Publisher for the /estop topic
        self.publisher = self.create_publisher(String, '/estop', 10)

        # Timer to periodically check the e-stop status
        self.timer = self.create_timer(0.1, self.check_estop_status)

        # Variable to track the flashing state
        self.flashing = False
        self.flash_state = False

    def check_estop_status(self):
        # Check the state of the e-stop pin
        if self.estop.value:  # Pin is receiving 3.3V
            self.publish_estop_status("safe")
            self.stop_flashing()
        else:  # Pin is not receiving 3.3V
            self.publish_estop_status("unsafe")
            self.start_flashing()

    def publish_estop_status(self, status):
        # Publish the e-stop status
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        self.get_logger().info(f'E-Stop status: {status}')

    def start_flashing(self):
        # Start flashing the LED
        if not self.flashing:
            self.flashing = True
            self.timer_flash = self.create_timer(0.5, self.flash_led)  # 500ms flash interval

    def stop_flashing(self):
        # Stop flashing the LED
        if self.flashing:
            self.flashing = False
            self.timer_flash.cancel()
            self.led.off()  # Ensure the LED is off when flashing stops

    def flash_led(self):
        # Toggle the LED state
        self.flash_state = not self.flash_state
        if self.flash_state:
            self.led.on()
        else:
            self.led.off()

def main(args=None):
    rclpy.init(args=args)
    estop_monitor = EStopMonitor()

    try:
        rclpy.spin(estop_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        estop_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


