#!/usr/bin/env python

#from picamera.array import PiYUVArray
from picamera import PiCamera
from PIL import Image
import time 
import numpy
import threading
import datetime
import os
import io
import math
import fractions
import json

class TimeLaps(object):
    def __init__(self, resolution=(2592, 1944), fps=1, delay=360, 
                 folder=".", avg_gains=200, avg_speed=5, config_file="parameters.json"):
        self.resolution = resolution
        self.fps = fps
        self.delay = delay
        self.folder = os.path.abspath(folder)
        self.avg_gains = avg_gains # time laps over which average gains
        self.avg_speeds = avg_speed # time laps over which average speed
        self.config_file = os.path.abspath(config_file)
        self.red_gains = []
        self.blue_gains = []
        self.speeds = []
	self.load_config()
        self.camera = PiCamera(resolution=self.resolution, framerate=self.fps) 
        self.camera.awb_mode = "auto"
        self.camera.iso = 0
        self.camera.shutter_speed = 0
        self.last_gain = 1
        self.last_speed = 0
        self.last_wb = (1,1)
        self.next_speed = 1
        self.next_wb = (1,1)
        self.last_img = time.time()
        self.next_img = self.last_img + self.delay

    def __del__(self):
        self.camera.close()
        self.camera = None

    def load_config(self):
        if os.path.isfile(self.config_file):
            with open(self.config_file) as jsonfile:
                dico = json.load(jsonfile)
            self.red_gains = dico.get("red_gains", self.red_gains)
            self.blue_gains = dico.get("blue_gains", self.blue_gains)
            self.speeds = dico.get("speeds", self.speeds)

    def save_config(self):
        dico = {"red_gains": self.red_gains, 
                "blue_gains": self.blue_gains,
                "speeds": self.speeds}

        with open(self.config_file,"w") as jsonfile:
            jsonfile.write(json.dumps(dico, indent=2))

    def save(self, ary, name):
        "save an array"
        Image.fromarray(ary).save(name)
    
    def capture(self):
        """Take a picture with the camera, if there is enough light
        """
        if not self.is_valid():
            print("Skipped, low light Speed %.3f/iso%.0f"%(self.last_speed, self.last_gain))
            return     
        ns = self.next_speed
        iso = 100
        while (ns < 4) and (iso < 800):
            iso *= 2
            ns *= 2
        if ns < 1.0:
            ns = 1.0
        if iso > 800:
            iso = 800
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = self.next_wb
        self.camera.iso = iso
        self.camera.shutter_speed = int(round(1e6/ns))
        self.last_img = time.time()
        self.next_img = self.last_img + self.delay
        fname = datetime.datetime.fromtimestamp(self.last_img).strftime("%Y-%m-%d-%Hh%Mm%Ss.jpg")
        print("%s s/%.3f iso %i, expected speed=%.3f R=%.3f B=%.3f"%(fname, ns, iso, self.next_speed, self.next_wb[0], self.next_wb[1]))
        self.camera.capture(fname)
        self.camera.awb_mode = "auto"
        self.camera.iso = 0
        self.camera.shutter_speed = 0

    def measure(self):   
        my_stream = io.BytesIO()
        self.camera.capture(my_stream, 'jpeg')
        r,b = self.camera.awb_gains
        self.last_speed = 1.0e6 / self.camera.exposure_speed
        self.last_gain = 1.0 * self.camera.analog_gain * self.camera.digital_gain
        if not self.is_valid():
            return
        r = 1.0 * r
        b = 1.0 * b
        self.last_wb = (r, b)
        ls = self.last_speed / self.last_gain
        self.red_gains.append(r)
        self.blue_gains.append(b)
        self.speeds.append(math.log(ls, 2.0))
        if len(self.red_gains) > self.avg_gains:
            red_gains = red_gains[-self.avg_gains:]
            blue_gains = blue_gains[-self.avg_gains:]
        if len(self.speeds) > self.avg_speeds:
           self.speeds = self.speeds[-self.avg_speeds:]
        rg = sum(self.red_gains)/len(self.red_gains)
        bg = sum(self.blue_gains)/len(self.blue_gains)
        ns = 2.0 ** (sum(self.speeds)/len(self.speeds))
        print("len %s/%s, \t S %.3f/iso%.0f -> %.3f \t R %.3f-> %.3f \t B %.3f -> %.3f"%(len(self.red_gains),len(self.speeds), self.last_speed, 100.*self.last_gain, ns, r, rg, b, bg))
        self.next_speed = ns
        self.next_wb = rg,bg
        self.save_config()

    def is_valid(self):
       return self.last_speed/self.last_gain >=1.5 
    
if __name__ == "__main__":
    tl = TimeLaps()
    print("Warmup ...")
    time.sleep(2)
    tl.measure()
    while True:
        tl.capture()
        while time.time() < tl.next_img:
            time.sleep(2.0*tl.delay/tl.avg_speeds)
            tl.measure()

