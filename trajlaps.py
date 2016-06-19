#!/usr/bin/env python
from __future__ import division, print_function
from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image
import time 
import numpy
import threading
import datetime
import os
import io
import math
from fractions import Fraction
import json
import pyexiv2
from collections import namedtuple

position = namedtuple("pan", "tilt")

trajectory={
"delay": 60,
"avg_awb":200,
"avg_speed":25,
"avg_speed_nb_img":2,
"trajectory": [
 {"tilt":45,
  "pan":90,
  "stay": 10,
  "move": 10},
 {"tilt":35,
  "pan":70,
  "stay": 10,
  "move": 10},
 {"tilt":00,
  "pan":+20,
  "stay": 10,
  "move": 10},
 {"tilt":00,
  "pan":-40,
  "stay": 10,
  "move": 10},
 ]
}
class Trajectory(object):
    def __init__(self, config=None, json_file=None):
        self.start_time = time.time()
        self.config = config
        if json_file:
            self.load_config(json_file)

    def load_file(self, filename):
        with open(filename) as jf:
            data = jf.read()
        config = json.load(data)
        if "trajectory" in config:
           self.config = config["trajectory"]
        else:
           self.config = config
    def goto(self, when):
        """move the camera to the position it need to be at a given timestamp"""
    def calc_pos(self, when):
        """Calculate the position it need to be at a given timestamp"""
        last_pos = position(0,90)
        remaining = when
        last_timestamp = 0
        for point in self.config:
            next_pos = position(point.get("pan",0), point.get("tilt",0))
            remaining -= point.get("move")
            if remaining < 0:
                break
            remaining -= point.get("stay")
            if remaining < 0:
                return next_pos
            last_timestamp = remaining
        else:
            #we ran out pof points: stay on the last
            return next_pos
        print(remaining)
        delta = last_timestamp - remaining
        delta_p = next_pos.pan - last_pos.pan
        delta_t = next_pos.tilt - last_po.tilt
        adv = - remaining / delta
        tilt = last_po.tilt + adv * delta_t
        pan = last_pos.pan + adv * delta_p
        return position(pan,tilt)

class TimeLaps(object):
    def __init__(self, resolution=(2592, 1944), fps=1, delay=360, 
                 folder=".", avg_gains=200, avg_speed=25, config_file="parameters.json"):
        self.start_time = time.time()
        self.resolution = resolution
        self.fps = fps
        self.delay = delay
        self.folder = os.path.abspath(folder)
        self.avg_gains = avg_gains # time laps over which average gains
        self.avg_speeds = avg_speed # time laps over which average speed
        self.avg_speed_nb_img = 3.0
        self.config_file = os.path.abspath(config_file)
        self.red_gains = []
        self.blue_gains = []
        self.speeds = []
	self.load_config()
        self.camera = PiCamera(resolution=self.resolution, framerate=self.fps) 
        self.raw = PiRGBArray(self.camera)
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
        self.raw.close()
        self.camera.close()
        self.raw = None
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
            self.last_img = time.time()
            self.next_img = self.last_img + self.delay
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
        self.camera.capture(self.raw, "rgb")
	Image.fromarray(self.raw.array).save(fname)
        exif = pyexiv2.ImageMetadata(fname)
        exif.read()
        exif["Exif.Photo.ExposureTime"] = Fraction(1, int(round(ns)))
        exif["Exif.Photo.ISOSpeedRatings"] = iso
        exif.write(preserve_timestamps=True)
        self.raw.truncate(0)
        self.camera.awb_mode = "auto"
        self.camera.iso = 0
        self.camera.shutter_speed = 0

    def measure(self):   
        self.camera.capture(self.raw, 'rgb')
        self.raw.truncate(0)
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
            self.red_gains = self.red_gains[-self.avg_gains:]
            self.blue_gains = self.blue_gains[-self.avg_gains:]
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
#    tl = TimeLaps()
#    print("Warmup ...")
#    time.sleep(2)
#    tl.measure()
#    while True:
#        tl.capture()
#        while time.time() < tl.next_img:
#            time.sleep(1.0*tl.avg_speed_nb_img*tl.delay/tl.avg_speeds)
#            tl.measure()
    pass
