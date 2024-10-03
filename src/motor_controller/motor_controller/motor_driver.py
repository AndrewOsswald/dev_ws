import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, PWMOutputDevice
from time import sleep

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # Create a subscriber to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)  # Queue size

    def cmd_vel_callback(self, msg):
        # Extract the linear and angular velocities
        linear_velocity = msg.linear.x  # Forward/backward speed
        angular_velocity = msg.angular.z  # Rotational speed

        # Use these values to drive your motor
        self.drive_motor(linear_velocity, angular_velocity)

    def drive_motor(self, linear_velocity, angular_velocity):
        # Implement your motor driving logic here
        # For example, translate the velocities into PWM signals

        self.get_logger().info(f'Driving motor - Linear: {linear_velocity}, Angular: {angular_velocity}')
        # Add code to control motor hardware here




        # Define GPIO pins
        IN1 = 14
        IN2 = 15
        ENA = 12  # This will be used for speed control via PWM

        # Set up the motor and PWM speed control
        motor = Motor(forward=IN1, backward=IN2)
        pwm = PWMOutputDevice(ENA)

        def set_motor(direction, speed):
            """
            Set motor direction and speed.
            :param direction: 'forward' or 'backward'
            :param speed: Speed of the motor (0-100%)
            """
            if direction == 'forward':
                motor.forward()
            elif direction == 'backward':
                motor.backward()
            else:
                raise ValueError("Direction must be 'forward' or 'backward'")

            # Set speed (speed must be a float between 0.0 and 1.0 for gpiozero)
            pwm.value = speed / 100.0

        def stop_motor():
            """
            Stop the motor by setting the speed to 0% and stopping the motor.
            """
            motor.stop()
            pwm.value = 0

        try:

            # Example usage
            print("Running motor forward at 50% speed")
            set_motor('forward', 50)
            sleep(10)

            #print("Running motor backward at 50% speed")
            #set_motor('backward', 50)
            #sleep(5)

            print("Stopping motor")
            stop_motor()
            sleep(1)

        finally:
            # Clean up GPIO settings
            stop_motor()



            

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()