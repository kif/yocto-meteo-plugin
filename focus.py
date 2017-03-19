#!/usr/bin/env python
from __future__ import division, print_function
from picamera import PiCamera
from picamera.array import PiYUVArray
from PIL import Image
import time 
import numpy
import threading
from scipy.ndimage import filters
import datetime
import os
import sys
import io
import math
import json

def blur_metric(data):
    """Measure the bluriness of an image"""
    DFh = abs(filters.sobel(data, axis=1))
    DFv = abs(filters.sobel(data, axis=0))
    Bv = filters.uniform_filter1d(data, 9, axis=0)
    Bh = filters.uniform_filter1d(data, 9, axis=1)
    DBh = abs(filters.sobel(Bh, axis=1))
    DBv = abs(filters.sobel(Bv, axis=0))
    Vv = numpy.maximum(0, DFv - DBv)
    Vh = numpy.maximum(0, DFh - DBh)
    sFv = DFv.sum()
    sFh = DFh.sum()
    sVv = Vv.sum()
    sVh = Vh.sum()
    bFv = (sFv-sVv)/sFv
    bFh = (sFh-sVh)/sFh
    return max(bFv, bFh)

    
if __name__ == "__main__":
    with PiCamera() as camera:
        #camera.resolution = (2592, 1944)
        camera.resolution = (1296, 972)
        with PiYUVArray(camera) as raw:
            print("Warmup ...")
            time.sleep(2)
            while True:
                f = camera.capture(raw, 'yuv')
                bw = raw.array[...,0].astype("float32")
                print(blur_metric(bw))
                raw.truncate(0)




