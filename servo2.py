import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=32
GPIO.setup(servo_pin,GPIO.OUT)
p=GPIO.PWM(servo_pin,50)
p.start(7.5)

try:
    while True:
	a = input("Enter degree between 0 and 180: ")
	if a<0 or a>180:
		a = input("Enter degree between 0 and 180: ")
	deg = round((a*10)/180,3) + 2.500
	print(deg)
	p.ChangeDutyCycle(deg)
	time.sleep(1)
	print("Rotated "+str(a)+" degrees")

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
