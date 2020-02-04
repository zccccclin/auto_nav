import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=32
GPIO.setup(servo_pin,GPIO.OUT)
p=GPIO.PWM(servo_pin,50)
p.start(7.5)

def serv():
    a = int(input("Enter degree: "))
    deg = round((a/180)*10,3)+2.5
    p.ChangeDutyCycle(deg)
    time.sleep(1)

try:
    while True:
        serv()
        
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
