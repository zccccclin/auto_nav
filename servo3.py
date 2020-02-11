import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=32
GPIO.setup(servo_pin,GPIO.OUT)
p=GPIO.PWM(servo_pin,50)
<<<<<<< HEAD:servo2.py
c = float(input("Enter start angle, between 0 and 180: "))
p.start(round(c*10/180,3) +2.5)
=======
#p.start(7.5)

def serv1():
    a = float(input("Enter degree between 0 and 180: "))
    while a<0 or a>180:
        a = float(input("Enter degree between 0 and 180: "))
    deg =round(a/18) + 2.5
    p.start(deg)
    print("Rotated to "+str(a)+" degrees")

>>>>>>> d3ba38371058e049faf0ee48724bab0421e86815:servo3.py

def serv():
    a = float(input("Enter degree between 0 and 180: "))
    while a<0 or a>180:
	a = float(input("Enter degree between 0 and 180: "))
<<<<<<< HEAD:servo2.py
    deg =round( a*10/180,3) + 2.5
=======
    deg =round(a/18) + 2.5
>>>>>>> d3ba38371058e049faf0ee48724bab0421e86815:servo3.py
    p.ChangeDutyCycle(deg)
#    time.sleep(1)
    print("Rotated to "+str(a)+" degrees")

<<<<<<< HEAD:servo2.py
=======
serv1()

>>>>>>> d3ba38371058e049faf0ee48724bab0421e86815:servo3.py
try:
    while True:
        serv()
        b = input('Would you like to try again: Enter 1 or 2, 1 = yes, 2 = no ')
        if b == 2:
	    break
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
