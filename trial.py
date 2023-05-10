import time
from lib2to3.pgen2 import driver
from time import sleep
import cv2
import pandas as pd
import labels as label
import numpy as np
from PIL import Image
from keras import models
import webbrowser
from ctypes import cast, POINTER
from gpiozero import AngularServo
from gpiozero import Servo
import RPi.GPIO as GPIO
import serial
from threading import Thread
from threading import Timer
from PIL import Image

#set up the GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

#set up PWM pins
stopperServo = GPIO.PWM(18,50)
mainServo = GPIO.PWM(23,50)
firServo = GPIO.PWM(24,50)
secServo = GPIO.PWM(25,50)

stopperServo.start(0)
mainServo.start(0)
firServo.start(0)
secServo.start(0)

coffeeQuality = 4

#open file and loading model
classNames = []
classFile = 'labels.txt'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
print(classNames)
model = models.load_model('model_cnn_beans.h5')

#For identifying the quality
def scroll(className):
    if className == 0:
        name = 'Grade 1'
    if className == 1:
        name = 'Grade 2'
    if className == 2:
        name = 'Grade 3'
    if className == 3:
        name = 'Grade 4'

#move the servo to custom position
def move_to_position(pwm, target_position):
    #calculate the duty cycle for the target position
    duty_cycle = 2.5 + (target_position/180) * (12.5-2.5)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1) #wait for servo to move

#change position for servo 
def gOneServo():
    move_to_position(stopperServo, 20)
    move_to_position(mainServo, 20)
    move_to_position(firServo, 20)
    move_to_position(secServo, 0)

def gTwoServo():
    move_to_position(stopperServo, 20)
    move_to_position(mainServo, 20)
    move_to_position(firServo, 20)
    move_to_position(secServo, 0)

def gThreeServo():
    move_to_position(stopperServo, 80)
    move_to_position(mainServo, 80)
    move_to_position(firServo, 0)
    move_to_position(secServo, 80)

def gFourServo():
    move_to_position(stopperServo, 80)
    move_to_position(mainServo, 80)
    move_to_position(firServo, 0)
    move_to_position(secServo, 80)

def gServoReset():
    move_to_position(stopperServo, 0)
    move_to_position(mainServo, 0)
    move_to_position(firServo, 0)
    move_to_position(secServo, 0)

gServoReset()

def camera():
    #for the camera
    video = cv2.VideoCapture(0)
    data = np.ndarray(shape=(1, 64, 64, 3), dtype=np.float32)

    while True:
        _, frame = video.read()
        im = Image.fromarray(frame, 'RGB')
        im = im.resize((64, 64))
        img_array = np.array(im)

        img_array = (img_array.astype(np.float32) / 127.0) - 1
        data[0] = img_array

        cv2.imshow("Camera 1", frame)
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
            
        quality = model.predict(data)
        coffeeQuality = quality.argmax()

        if coffeeQuality == 0:
            name = 'Grade 1'
        if coffeeQuality == 1:
            name = 'Grade 2'  
    
        if coffeeQuality == 2:
            name = 'Grade 3'

        if coffeeQuality == 3:
            name = 'Grade 4'

        print("Quality: ", name)

    video.release()   

sleep(1)

#for sorting of the beans
def sorting():
    if coffeeQuality == 0:
        gOneServo()

    if coffeeQuality == 1:
        gTwoServo()
    
    if coffeeQuality == 2:
        gThreeServo()

    if coffeeQuality == 3:
        gFourServo()
    else:
        gServoReset()
    
#threading to reduce delay between camera and sorting
threadCamera = Thread(target=camera)
threadSorting = Thread(target=sorting)
while True:
    threadCamera.start()
    threadSorting.start()



