import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

test_point = 40

GPIO.setup(test_point,GPIO.OUT)

try:
    while True:
        GPIO.output(test_point,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(test_point,GPIO.LOW)
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
