import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

test_point = 40

GPIO.setup(test_point,GPIO.OUT)

try:
    while True:
        a = str(input('Enter command: open/close ')
        if a == 'open':
            GPIO.output(test_point,GPIO.HIGH)
            time.sleep(1)
        elif a == 'close':
            GPIO.output(test_point,GPIO.LOW)
            time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()