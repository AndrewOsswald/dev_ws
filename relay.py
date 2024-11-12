from gpiozero import LED

# Set up pin 10 as output
pin_10 = LED(10)

while True:
    # Get command from the console
    command = input("Enter 'on' to turn on the pin, 'off' to turn them off, or 'exit' to quit: ").strip().lower()

    if command == "on":
        # Turn on pin 10
        pin_10.on()
        print("Pins 10 is now ON.")
        
    elif command == "off":
        # Turn off pin 10
        pin_10.off()
        print("Pin 10 is now OFF.")
        
    elif command == "exit":
        # Turn off pin
        pin_10.off()
        print("Exiting program and turning off pin.")
        break
    
    else:
        print("Invalid command. Please enter 'on', 'off', or 'exit'.")
