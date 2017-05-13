#!/usr/bin/python

import time
from mma8451 import MMA8451, RANGE_2G, BW_RATE_6_25HZ
from threading import Thread, Event, Semaphore
from collections import namedtuple

Gravity = namedtuple("Gravity",["x", "y", "z"])


class Accelerometer(Thread):
    def __init__(self, still=Event()):
        Thread.__init__(self, name="Accelerometer")
        self.sem = Semaphore()
        self.still_event = still
        self.mma8451 = MMA8451(sensor_range=RANGE_2G,
                               data_rate=BW_RATE_6_25HZ, 
                               debug=False)
        self.mma8451.set_resolution()
        self.history = []
        self.stored = None

    def run(self):
        "Main thread's activity: collect gravity information"
        while True:
            if self.still_event.is_set():
                self.collect()
            else:
                self.average()
                self.reset()
                self.still_event.wait()
            time.sleep(0.1)

    def get(self):
        """return the gravity vector"""
        if self.still_event.is_set():
            return self.average()
        else:
            return self.stored

    def average(self):
        if not self.history:
            return self.stored
        with self.sem:
            x,y,z = 0
            for i, g in enumerate(self.history):
                x += g.x
                y += g.y
                z += g.z
            print("average over", i)
            self.stored = Gravity(x/(i+1.0), y/(i+1.0), z/(i+1.0))
        return self.stored

    def reset(self):
        with self.sem:
            self.history = []

    def collect(self):
        "collect a value and adds it to the history"
        axes = self.mma8451.get_axes_measurement()
        with self.sem:
            if self.still_event.is_set():
                self.history.append(Gravity(axes["x"], axes['y'], axes['z']))

if __name__ == "__main__":
    mma8451 = MMA8451(sensor_range=RANGE_2G,data_rate=BW_RATE_6_25HZ, debug=False)
    mma8451.set_resolution()
    while True:
        axes = mma8451.get_axes_measurement()
        print("g x = %.3fm/s2    y = %.3fm/s2    z = %.3fm/s2" % 
                             (axes['x'], axes['y'], axes['z']))
                #print("Position = %d   x = %.3fm/s2    y = %.3fm/s2    z = %.3fm/s2" % 
        #     (mma8451.get_orientation(),axes['x'], axes['y'], axes['z']))
        time.sleep(0.1)

