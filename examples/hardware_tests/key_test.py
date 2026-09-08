# button_test_simple.py
import RPi.GPIO as GPIO
import time


BUTTON_PIN = 17


GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Button test program started")
print("Press button connected to GPIO", BUTTON_PIN)
print("Press CTRL+C to exit")

try:
    while True:
        
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            print("Button pressed")
            time.sleep(0.1)  
        else:
            time.sleep(0.1)  
except KeyboardInterrupt:
    print("\nProgram exited")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up")
