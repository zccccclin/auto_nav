import cv2
import numpy as np
import time 
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
servo_pin=31

GPIO.setup(servo_pin, GPIO.OUT)
p=GPIO.PWM(servo_pin, 50)
p.start(7.5)

cap = cv2.VideoCapture(0)
# Set camera resolution
cap.set(3, 480)
cap.set(4, 320)
_, frame = cap.read()
rows, cols, _ = frame.shape

try:
	while True:
        	_, frame = cap.read()
        	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        	# red color
        	low_red = np.array([161, 155, 84])
        	high_red = np.array([179, 255, 255])
        	red_mask = cv2.inRange(hsv_frame, low_red, high_red)
        	_, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        	contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        	for cnt in contours:
        		(x, y, w, h) = cv2.boundingRect(cnt)
        		x_medium = int((x + x + w) / 2)
        		break
    		cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
		
		if x_medium<center-30:
			p.ChangeDutyCycle(12.5)
			time.sleep(1)
		elif x_medium<center+30:
			p.ChangeDutyCycle(2.5)
			time.sleep(1)
		else:
			p.ChangeDutyCycle(7.5)
			time.sleep(1)
except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
	break
