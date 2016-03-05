#!/usr/bin/env python

from picamera.array import PiYUVArray
from picamera import PiCamera
from PIL import Image
import time 
import numpy
import threading

#resolution = (2592, 1944)
resolution = (1296, 972)
fps = 2
bining = (8, 4)
thres = 20
points = 20
delay = 3600


def save(ary, name):
    "save an array"
    Image.fromarray(ary).save(name)


with PiCamera() as camera:
    camera.resolution = resolution
    camera.framerate = fps
    t0 = time.time()
    last = None
    timestamp = 0
    print("Warmup ...")
    time.sleep(2)
    fps = 0
    with PiYUVArray(camera, size=resolution) as raw:
        for f in camera.capture_continuous(raw, format="yuv", use_video_port=True):
#         while True:
#            f = camera.capture(raw, 'yuv')
            frame = raw.array
            nb = frame[...,0]
            nb.shape = resolution[1]//bining[1], bining[1], resolution[0]//bining[0], bining[0]
            new = nb.mean(axis=-1, dtype=int).mean(axis=1, dtype=int)
            if last is not None:
                delta = abs(last-new)
                keep = (delta>thres).sum()
            else:
                keep = 0
            if keep>points or (time.time()-timestamp)>delay:
                timestamp = time.time()
                cent = ("%.2f"%timestamp).split(".")[1]
                name = time.strftime("%Y-%m-%d-%Hh%Mm%Ss",time.gmtime(timestamp))+cent+".jpg"
                print("%s last fps: %.2f, count: %i"%(name, fps, keep))
                savethread = threading.Thread(target=save, args=(raw.rgb_array, name))
                savethread.start()
            last = new
            raw.truncate(0)
            t1 = time.time()
            fps = 1.0 / (t1 - t0)
            #print("fps= %.2f"%(1.0/(t1-t0)))
            t0 = t1

