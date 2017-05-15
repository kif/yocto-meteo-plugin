#!/usr/bin/python
from __future__ import division
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 50hz, good for servos.
# gives a period of 20ms/cycle
FREQ = 300
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
        ticks = self.calc(angle)
        #print("ticks: %s"%ticks)
        pwm.set_pwm(self.pin, 0, int(round(ticks)))

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


class M15S(SG90):
    SPEC = { -90: 482,
            180: 2604,}


class MG90s(SG90):
    SPEC={  4: 581,#us
          176: 2380}


class TSS11MGB(Servo):
    SPEC={  2.17: 691.732,#deg->us
            175.23: 2685.55}
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

    def calc(self, angle):
        if self.reverse:
            angle=-angle
        if self.offset:
            angle -= self.offset
        #Protect for out of bounds ...
        angles = list(self.SPEC.keys())
        angle = max(angle, min(angles))
        angle = min(angle, max(angles))
        #print(angle)
        return int(round(self.inter + self.slope*angle))

pan = M15S(14, reverse=True, offset=-90)
tilt = SG90(15, reverse=False, offset=-90)
mg90 = MG90s(13, reverse=False, offset=0)
tss = TSS11MGB(12)
