import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=32
GPIO.setup(servo_pin,GPIO.OUT)
p=GPIO.PWM(servo_pin,50)
c = float(input("Enter start angle, between 0 and 180: "))
p.start(round(c*10/180,3) +2.5)

def serv():
    a = float(input("Enter degree between 0 and 180: "))
    if a<0 or a>180:
	a = float(input("Enter degree between 0 and 180: "))
    deg =round( a*10/180,3) + 2.5
    p.ChangeDutyCycle(deg)
    time.sleep(1)
    print("Rotated to "+str(a)+" degrees")

try:
    while True:
        serv()
        b = input('Would you like to try again: Enter 1 or 2, 1 = yes, 2 = no ')
        if b == 2:
	    break
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
