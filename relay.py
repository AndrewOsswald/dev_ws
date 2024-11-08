from gpiozero import LED

# Set up pins 9 and 10 as outputs
pin_9 = LED(9)
pin_10 = LED(10)

while True:
    # Get command from the console
    command = input("Enter 'on' to turn on the pins, 'off' to turn them off, or 'exit' to quit: ").strip().lower()

    if command == "on":
        # Turn on both pins
        pin_9.on()
        pin_10.on()
        print("Pins 9 and 10 are now ON.")
        
    elif command == "off":
        # Turn off both pins
        pin_9.off()
        pin_10.off()
        print("Pins 9 and 10 are now OFF.")
        
    elif command == "exit":
        # Turn off pins and exit the program
        pin_9.off()
        pin_10.off()
        print("Exiting program and turning off pins.")
        break
    
    else:
        print("Invalid command. Please enter 'on', 'off', or 'exit'.")
