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
import sys
import io
import math
from fractions import Fraction
import json
import pyexiv2
from collections import namedtuple
import Adafruit_PCA9685
from argparse import ArgumentParser


Position = namedtuple("Position", ("pan", "tilt"))


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
        self.config = config or []
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.servo_id_tilt = 15
        self.servo_id_pan = 14
        self.default_pos = Position(0, 90)

        if json_file:
            self.load_config(json_file)

    @property
    def duration(self):
        return sum( i.get("stay") + i.get("move") for i in self.config)
        
    def __repr__(self):
        return "Trajectory over %i points lasting %.3fs"%(len(self.config), self.duration)

    def load_config(self, filename):
        with open(filename) as jf:
            data = jf.read()
        config = json.loads(data)
        if "trajectory" in config:
           self.config = config["trajectory"]
        else:
           self.config = config

    def goto(self, when):
        """move the camera to the position it need to be at a given timestamp"""
        pos = self.calc_pos(when)
        self.goto_pos(pos)
        return pos

    def goto_pos(self, pos):
        pan, tilt = pos
        self.pwm.set_pwm(self.servo_id_pan, 0, self.angle(pan))
        self.pwm.set_pwm(self.servo_id_tilt, 0, self.angle(tilt))
 
    def calc_pos(self, when):
        """Calculate the position it need to be at a given timestamp"""
        last_pos = self.default_pos
        last_timestamp = remaining = when
        if remaining <= 0:
            return last_pos
        for point in self.config:
            next_pos = Position(point.get("pan",0), point.get("tilt",0))
            #print(last_pos, next_pos)
            remaining -= point.get("move")
            if remaining < 0:
                break
            remaining -= point.get("stay")
            if remaining < 0:
                return next_pos
            last_timestamp = remaining
            last_pos = next_pos
        else:
            #we ran out pof points: stay on the last
            return next_pos
        delta = last_timestamp - remaining
        delta_p = next_pos.pan - last_pos.pan
        delta_t = next_pos.tilt - last_pos.tilt
        adv = last_timestamp / delta
        tilt = last_pos.tilt + adv * delta_t
        pan = last_pos.pan + adv * delta_p
        #print(last_timestamp, remaining, delta, adv)
        return Position(pan,tilt)

    @staticmethod
    def angle(val):
        '''calculate the pwm for requested angle
        angle goes from -90 to +90 deg'''
        # Configure min and max servo pulse lengths
        servo_min = 150  # Min pulse length out of 4096
        servo_max = 600  # Max pulse length out of 4096
        return int(round(servo_min + (90.0 - val) * (servo_max - servo_min) / 180.0))


class TimeLaps(object):
    def __init__(self, resolution=(2592, 1944), fps=1, delay=360, 
                 folder=".", avg_awb=200, avg_speed=25, config_file="parameters.json"):
        self.resolution = resolution
        self.fps = fps
        self.delay = delay
        self.folder = os.path.abspath(folder)
        self.avg_awb = avg_awb # time laps over which average gains
        self.avg_speed = avg_speed # time laps over which average speed
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
        self.position = Position(0,0)
        self.start_time = self.last_img = time.time()
        self.next_img = self.last_img + self.delay
        self.trajectory = Trajectory(json_file=config_file)
        self.position = self.trajectory.goto(self.delay)
        self.load_config() 

    def __del__(self):
        self.raw.close()
        self.camera.close()
        self.raw = None
        self.camera = None
        self.trajectory = None

    def load_config(self):
        if os.path.isfile(self.config_file):
            with open(self.config_file) as jsonfile:
                dico = json.load(jsonfile)
            self.red_gains = dico.get("red_gains", self.red_gains)
            self.blue_gains = dico.get("blue_gains", self.blue_gains)
            self.speeds = dico.get("speeds", self.speeds)
            self.delay = dico.get("delay", self.delay)
            self.avg_speed_nb_img = dico.get("avg_speed_nb_img", self.avg_speed_nb_img)
            self.avg_speed = dico.get("avg_speed", self.avg_speed)
            self.avg_awb = dico.get("avg_awb", self.avg_awb)
            
    def save_config(self):
        dico = {"red_gains": self.red_gains, 
                "blue_gains": self.blue_gains,
                "speeds": self.speeds,
                "trajectory": self.trajectory.config,
                "avg_awb": self.avg_awb,
                "avg_speed": self.avg_speed,
                "self.avg_speed_nb_img": self.avg_speed_nb_img,
                "delay": self.delay
                }

        with open(self.config_file,"w") as jsonfile:
            jsonfile.write(json.dumps(dico, indent=2))

    def save(self, ary, name, exp_speed=125, iso=100):
        "save an array"
        Image.fromarray(ary).save(name)
        exif = pyexiv2.ImageMetadata(name)
        exif.read()
        exif["Exif.Photo.ExposureTime"] = Fraction(1, int(round(exp_speed)))
        exif["Exif.Photo.ISOSpeedRatings"] = iso
        exif.comment = "pan=%s, tilt=%s"%self.position
        #exif["Exif.MakerNote.Tilt"] = self.position.tilt
        exif.write(preserve_timestamps=True)
    
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
        self.save(self.raw.array, fname, ns, iso)
        self.raw.truncate(0)
        self.camera.awb_mode = "auto"
        self.camera.iso = 0
        self.camera.shutter_speed = 0
        self.position = self.trajectory.goto(self.next_img - self.start_time)

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
        if len(self.red_gains) > self.avg_awb:
            self.red_gains = self.red_gains[-self.avg_awb:]
            self.blue_gains = self.blue_gains[-self.avg_awb:]
        if len(self.speeds) > self.avg_speed:
           self.speeds = self.speeds[-self.avg_speed:]
        rg = sum(self.red_gains)/len(self.red_gains)
        bg = sum(self.blue_gains)/len(self.blue_gains)
        ns = 2.0 ** (sum(self.speeds)/len(self.speeds))
        print("len %s/%s, \t S %.3f/iso%.0f -> %.3f \t R %.3f-> %.3f \t B %.3f -> %.3f"%(len(self.red_gains),len(self.speeds), self.last_speed, 100.*self.last_gain, ns, r, rg, b, bg))
        self.next_speed = ns
        self.next_wb = rg,bg
        self.save_config()

    def is_valid(self):
       return self.last_speed/self.last_gain >=0.4 
    
if __name__ == "__main__":
    parser = ArgumentParser("trajlaps", 
                            description="TimeLaps over a trajectory")
    parser.add_argument("-j", "--json", help="config file")
    args = parser.parse_args()
    tl = TimeLaps(config_file=args.json)
    print("Warmup ...")
    time.sleep(2)
    tl.measure()
    while True:
        tl.capture()
        while time.time() < tl.next_img:
            time.sleep(1.0*tl.avg_speed_nb_img*tl.delay/tl.avg_speed)
            tl.measure()
    pass
