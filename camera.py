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



