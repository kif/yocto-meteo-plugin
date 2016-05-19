#!/usr/bin/python
# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import os
import time
import sys
from math import atan2, pi, floor, ceil
import numpy
import time

import picamera
cam = picamera.PiCamera()
# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
servo_tilt = 15
servo_pan = 14

lense_f = 4.0 #mm
camera_shape = 1944, 2592
cam.resolution = camera_shape[1], camera_shape[0]
pixel = 1.4e-3, 1.4e-3 #mm
half_size = [i*j/2 for i,j in zip(camera_shape, pixel)]
FOV = [ 360*atan2(i,lense_f)/pi for i in half_size]

overlap = 0.3
tilt_min = -90
tilt_max = 60
pan_min = -90
pan_max = 90

tilt_step = int(ceil((tilt_max-tilt_min)/(FOV[0]*(1-overlap))))

pos = []
last_pan = 1
for tilt in numpy.linspace(tilt_min, tilt_max, tilt_step):
    pan_step = int(ceil((pan_max-pan_min)*numpy.cos(numpy.deg2rad(tilt))/(FOV[1]*(1-overlap))))
    if last_pan < 0:
        for pan in numpy.linspace(pan_min, pan_max, pan_step):
            pos.append((tilt, pan))
    else:
        for pan in numpy.linspace(pan_max, pan_min, pan_step):
            pos.append((tilt, pan))
    last_pan = pan

print("Scanning positions: ")
print(pos)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def angle(val):
    '''calculate the pwm for requested angle
    angle goes from -90 to +90 deg'''
    return int(servo_min + (val + 90) * (servo_max - servo_min) / 180.)

now = time.strftime("%Y-%m-%dT%Hh%Mm%Ss")
print("Start scan at %s"%now)
os.mkdir(now)

for tilt, pan in pos:
    pwm.set_pwm(servo_tilt, 0, angle(tilt))
    pwm.set_pwm(servo_pan, 0, angle(pan))
    fn = os.path.join(now,"img_%+06.2f_%+06.2f.jpg"%(tilt, pan))
    print("    "+fn)
    time.sleep(1)
    cam.capture(fn)

print("Moving back to zenith")
pwm.set_pwm(servo_tilt, 0, angle(-90))
pwm.set_pwm(servo_pan, 0, angle(0))
