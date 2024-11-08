from gpiozero import LED
from time import sleep

# Set up pins 9 and 10 as outputs
pin_9 = LED(9)
pin_10 = LED(10)

# Turn on both pins
pin_9.on()
pin_10.on()

# Keep the signal active for 5 seconds
sleep(100)

# Turn off both pins
pin_9.off()
pin_10.off()
