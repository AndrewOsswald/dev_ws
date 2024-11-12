from gpiozero import PWMLED

# Set up pin 9 as a PWM output
pin_9 = PWMLED(9)

while True:
    # Get command from the console
    command = input("Enter 'on' to set PWM to 30%, 'off' to turn off, or 'exit' to quit: ").strip().lower()
    
    if command == "on":
        # Set PWM to 30% duty cycle
        pin_9.value = 0.3  # 30% duty cycle
        print("Pin 9 is now at 30% duty cycle.")
        
    elif command == "off":
        # Turn off the PWM (0% duty cycle)
        pin_9.off()
        print("Pin 9 is now OFF.")
        
    elif command == "exit":
        # Turn off PWM and exit the program
        pin_9.off()
        print("Exiting program and turning off pin 9.")
        break
    
    else:
        print("Invalid command. Please enter 'on', 'off', or 'exit'.")

