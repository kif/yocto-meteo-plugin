#!/usr/bin/python
# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import sys

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

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def angle(val):
    '''calculate the pwm for requested angle
    angle goes from -90 to +90 deg'''
    return servo_min + (val + 90) * (servo_max - servo_min) // 180

print('Moving servos of tilt-pan camera to zenith tilt=90 pan=0')
pwm.set_pwm(servo_pan, 0, angle(0))
pwm.set_pwm(servo_tilt, 0, angle(-90))
