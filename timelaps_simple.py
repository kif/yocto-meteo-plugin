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

resolution = (2592, 1944)
#resolution = (1296, 972)
fps = 1
delay = 360 # 10 images per hour
#delay = 5
folder = "."

nb_gains = 20 #transition over 2hours
config_file = "parameters.json"
if os.path.isfile(config_file):
    with open(config_file) as jsonfile:
        dico = json.load(jsonfile)
    red_gains = dico.get("red_gains", [])
    blue_gains = dico.get("blue_gains", [])
    speeds = dico.get("speeds", [])
else:
    red_gains = []
    blue_gains = []
    speeds = []


def save(ary, name):
    "save an array"
    Image.fromarray(ary).save(name)

def capture(cam):
    """
    Take a picture with the camera
    """
    fname = datetime.datetime.now().strftime("%Y-%m-%d-%Hh%Mm%Ss.jpg")
    print(fname)
    cam.capture(fname)

def calc_awb(camera, red_gains, blue_gains, speeds):
    camera.awb_mode = 'auto'
    camera.shutter_speed = 0
    my_stream = io.BytesIO()
    camera.capture(my_stream, 'jpeg')
    r,b = camera.awb_gains
    speed = camera.exposure_speed
    r = 1.0*r.numerator/r.denominator
    b = 1.0*b.numerator/b.denominator
    ls = (1.0*speed.numerator/speed.denominator)
    red_gains.append(r)
    blue_gains.append(b)
    speeds.append(math.log(ls, 2.0))
    if len(red_gains)>nb_gains:
        red_gains = red_gains[-nb_gains:]
        blue_gains = blue_gains[-nb_gains:]
        speeds = speeds[-nb_gains:]
    rg = sum(red_gains)/len(red_gains)
    bg = sum(blue_gains)/len(blue_gains)
    ns = max(1,int(round(math.pow(2.0, sum(speeds)/len(speeds)))))
    print("len %s, \t S %s -> %s \t R %s-> %s \t B %s -> %s"%(len(red_gains), ls, ns, r,rg, b, bg))
    camera.awb_mode = 'off'
    camera.awb_gains = rg, bg
    camera.shutter_speed = int(ns)
    with open(config_file,"w") as jsonfile:
        jsonfile.write(json.dumps({"red_gains":red_gains, "blue_gains": blue_gains, "speeds": speeds}, indent=2))

    
    

with PiCamera(resolution=resolution, framerate=fps) as camera:
    t0 = time.time()
    last = None
    timestamp = 0
#    camera.start_preview()
    print("Warmup ...")
    time.sleep(2)
    #capture(camera)
    calc_awb(camera, red_gains, blue_gains, speeds)
    red_gains += [red_gains[-1]] * (nb_gains - len(red_gains))
    blue_gains += [blue_gains[-1]] * (nb_gains - len(blue_gains))
    speeds += [speeds[-1]] * (nb_gains - len(speeds))
    while True:
        next = time.time() + delay
        calc_awb(camera, red_gains, blue_gains, speeds)
        capture(camera)
        while time.time() < next:
            time.sleep(1)

