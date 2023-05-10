import RPi.GPIO as GPIO
import time

#set up the GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

#set up PWM pins
pwm1 = GPIO.PWM(18,50)
pwm2 = GPIO.PWM(23,50)
pwm3 = GPIO.PWM(24,50)

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

#move the servo to custom position
def move_to_position(pwm, target_position):
    #calculate the duty cycle for the target position
    duty_cycle = 2.5 + (target_position/180) * (12.5-2.5)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1) #wait for servo to move

#move the three servo from 0 - 180 in 10 degree increments
for pos in range(0,180,60):
    move_to_position(pwm1, pos)
    move_to_position(pwm2, pos)
    move_to_position(pwm3, pos)

#move the three servo from 180 - 0 in 10 degree increments
for pos in range(180,-1,-60):
    move_to_position(pwm1, pos)
    move_to_position(pwm2, pos)
    move_to_position(pwm3, pos)

#move the three servo to 0 degree before cleaning up the GPIO
move_to_position(pwm1, 0)
move_to_position(pwm2, 0)
move_to_position(pwm3, 0)

#clean up GPIO
pwm1.stop()
pwm2.stop()
pwm3.stop()
GPIO.cleanup()