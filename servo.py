#!/usr/bin/python
from __future__ import division
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 50hz, good for servos.
# gives a period of 20ms/cycle
FREQ = 50
RESOLUTION = 4096
pwm.set_pwm_freq(FREQ)

class Servo(object):
    """Represent a servomotor"""
    def __init__(self, pin, offset=0, reverse=False):
        """
        :param pin: pin number on the PCA9685 card
        :param offset: 
        """
        self.pin = pin
        self.offset = offset
        self.reverse = reverse
        self.slope = 0
        self.inter = 0

    def move(self, angle):
        angle = self.calc(angle)
        pwm.set_pwm(self.pin, 0, int(round(angle)))

    def off(self):
        pwm.set_pwm(self.pin, 0, 0)

    def calc(self, angle):
        if self.reverse:
            angle=-angle
        if self.offset:
            angle -= self.offset
        return int(round(self.inter + self.slope*angle))


class SG90(Servo):
    SPEC={  4: 581,#us
            176: 2380}
    def __init__(self, pin, offset=0, reverse=False):
        Servo.__init__(self, pin, offset=offset, reverse=reverse)
        angles = list(self.SPEC.keys())
        amax = max(angles)
        amin = min(angles)
        tick = 1.0e6 / FREQ / RESOLUTION
        tickmax = self.SPEC[amax] / tick
        tickmin = self.SPEC[amin] / tick
        self.slope = (tickmax - tickmin) / (amax - amin)
        self.inter = tickmin - amin * self.slope 

pan = SG90(14, reverse=True, offset=-90)
tilt = SG90(15, reverse=True, offset=-90)

