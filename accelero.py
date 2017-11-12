#!/usr/bin/python

import logging
import time
from mma8451 import MMA8451, RANGE_2G, BW_RATE_6_25HZ, BW_RATE_1_56HZ
from threading import Thread, Event, Semaphore
from collections import namedtuple
import signal

logger = logging.getLogger("accelero")
Gravity = namedtuple("Gravity",["x", "y", "z"])


class Accelerometer(Thread):
    def __init__(self, sensor_range=RANGE_2G, data_rate=BW_RATE_1_56HZ, quit_event=None):
        Thread.__init__(self, name="Accelerometer")
        signal.signal(signal.SIGINT, self.quit)
        self.sem = Semaphore()
        self._can_record = Event()
        self._done_recording = Event()
        self._quit = quit_event or Event()

        self.mma8451 = MMA8451(sensor_range=sensor_range,
                               data_rate=data_rate, 
                               debug=False)
        self.mma8451.set_resolution()
        self.history = []
        self.stored = Gravity(1,1,1)
        self.resume()

    def run(self):
        "Main thread's activity: collect gravity information"
        while not self._quit.is_set():
            self._can_record.wait()
            try:
                self._done_recording.clear()
                self.collect()
            finally:
                self._done_recording.set()

    def quit(self, *arg, **kwarg):
        "quit the main loop and end the thread"
        self._quit.set()

    def pause(self):
        "pause the recording, wait for the current value to be saved"
        self._can_record.clear()
        self._done_recording.wait()
        return self.average(reset=True)
    
    def resume(self):
        "resume the recording"
        self._can_record.set()

    def get(self):
        """return the gravity vector"""
        if self._can_record.is_set():
            return self.average()
        else:
            return self.stored

    def average(self, reset=False):
        "average all values, stores them and return the average" 
        if not self.history:
            return self.stored
        with self.sem:
            x = 0.0
            y = 0.0
            z = 0.0
            for i, g in enumerate(self.history):
                x += g.x
                y += g.y
                z += g.z
            logger.debug("average over %s measurements", i)
            self.stored = Gravity(x/(i+1.0), y/(i+1.0), z/(i+1.0))
            if reset:
                self.history = []
        return self.stored

    def collect(self):
        "collect a value and adds it to the history"
        axes = self.mma8451.get_axes_measurement()
        g2 = axes["x"]**2+axes['y']**2+axes['z']**2
        #print(g2)
        if g2>90 and g2<100: #keep only if acc ~= g 
            with self.sem:
                self.history.append(Gravity(axes["x"], axes['y'], axes['z']))


if __name__ == "__main__":
    from math import atan2, pi
    acc = Accelerometer()
    acc.start()
    i = 0
    while True:
        i+=1
        grav = acc.get()
        print("Tilt %6.3f Yaw %6.3f %s"%(180.0 * atan2(-grav.y, -grav.z) / pi, 180.0 * atan2(-grav.x, -grav.z) / pi, grav))
        if i%10==0:
            print("pause")
            acc.pause()
        elif i%10==3:
            print("resume")
            acc.resume()
        time.sleep(1)

