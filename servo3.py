import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=32
GPIO.setup(servo_pin,GPIO.OUT)
p=GPIO.PWM(servo_pin,50)
p.start(7.5)

def serv():
    a = float(input("Enter degree between 0 and 180: "))
    while a<0 or a>180:
	a = float(input("Enter degree between 0 and 180: "))
    deg =round( a*10/180,5) + 2.5
    p.ChangeDutyCycle(deg)
    time.sleep(1)
    print("Rotated to "+str(a)+" degrees")


try:
    while True:
	serv()

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
