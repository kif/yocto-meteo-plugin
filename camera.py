#!/usr/bin/env python
from __future__ import division, print_function
import os
from collections import namedtuple, OrderedDict
import time
import threading
try:
    from Queue import Queue
except ImportError:
    from queue import Queue
import io
import logging
from fractions import Fraction
import numpy
from PIL import Image
import pyexiv2
from picamera import PiCamera
from scipy.signal import convolve, gaussian, savgol_coeffs
from exposure import lens
logger = logging.getLogger("camera")

ExpoRedBlue = namedtuple("ExpoRedBlue", ("ev", "dev", "over", "red", "green", "blue"))

class SavGol(object):
    "Class for Savitsky-Golay filtering"
    def __init__(self, order=2):
        "select the order of the filter"
        self.order = order
        self.cache = {} #len, filter

    def __call__(self, lst):
        "filter a list. the last having the more weight"
        l = len(lst)
        if l%2 == 0:
            lst = numpy.array(lst[1:])
            l -= 1
        else:
            lst = numpy.array(lst)
        if l not in self.cache:
            self.cache[l] = savgol_coeffs(l, self.order, pos=0)
        return numpy.dot(lst, self.cache[l])


savgol = SavGol()


class Frame(object):
    """This class holds one image"""
    INDEX = 0
    semclass = threading.Semaphore()
            # YUV conversion matrix from ITU-R BT.601 version (SDTV)
            #                  Y       U       V
    YUV2RGB = numpy.array([[1.164,  0.000,  1.596],                          # R
                           [1.164, -0.392, -0.813],                          # G
                           [1.164,  2.017,  0.000]], dtype=numpy.float32).T  # B

    def __init__(self, data):
        "Constructor"
        self.timestamp = time.time()
        with self.semclass:
            self.index = self.INDEX
            self.__class__.INDEX += 1
        self.data = data
        self.camera_meta = {}
        self.gravity = None
        self.position = None
        self.sem = threading.Semaphore()
        self._yuv = None
        self._rgb = None

    def save(self):
        "Save the data as YUV raw data"
        fname = self.get_date_time()+".yuv"
        with open(fname, "w") as f:
            f.write(self.data)
        logger.info("Saved YUV raw data %i %s", self.index, fname)

    def get_date_time(self):
        return time.strftime("%Y-%m-%d-%Hh%Mm%Ss", time.localtime(self.timestamp)) 

    @property
    def yuv(self):
        """Retrieve the YUV array"""
        if self._yuv is None:
            with self.sem:
                if self._yuv is None:
                    width, height = self.camera_meta.get("resolution", (640, 480))
                    fwidth = (width + 31) & ~(31)
                    fheight = (height + 15) & ~ (15)
                    ylen = fwidth * fheight
                    uvlen = ylen // 4
                    ary = numpy.frombuffer(self.data, dtype=numpy.uint8)
                    Y = ary[:ylen]
                    U = ary[ylen: - uvlen]
                    V = ary[-uvlen:]
                    # Reshape the values into two dimensions, and double the size of the
                    # U and V values (which only have quarter resolution in YUV4:2:0)
                    Y = Y.reshape((fheight, fwidth))
                    U = U.reshape((fheight // 2, fwidth // 2)).repeat(2, axis=0).repeat(2, axis=1)
                    V = V.reshape((fheight // 2, fwidth // 2)).repeat(2, axis=0).repeat(2, axis=1)
                    # Stack the channels together and crop to the actual resolution
                    self._yuv = numpy.dstack((Y, U, V))[:height, :width]
        return self._yuv

    @property
    def rgb(self):
        """retrieve the image a RGB array"""
        if  self._rgb is None:
            YUV = numpy.asarray(self.yuv, dtype=numpy.float32)
            with self.sem:
                if self._rgb is None:
                    YUV[:, :, 0] = YUV[:, :, 0] - 16.0  # Offset Y by 16
                    YUV[:, :, 1:] = YUV[:, :, 1:] - 128.0 # Offset UV by 128
                    # Calculate the dot product with the matrix to produce RGB output,
                    # clamp the results to byte range and convert to bytes
                    self._rgb = YUV.dot(self.YUV2RGB).clip(0, 255).astype(numpy.uint8)
        return self._rgb


class StreamingOutput(object):
    """This class handles the stream, it re-cycles a BytesIO and provides frames"""
    def __init__(self, size):
        """Constructor

        :param size: size of an image in bytes. 
        For YUV, it is 1.5x the number of pixel of the padded image.
        """
        self.size = size
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = threading.Condition()

    def write(self, buf):
        res = self.buffer.write(buf)
        if self.buffer.tell() >= self.size:
            #image complete
            self.buffer.truncate(self.size)
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            with self.condition:
                self.frame = Frame(self.buffer.getvalue())
                self.condition.notify_all()
            self.buffer.seek(0)
        else:
            print("Incomplete buffer of %i bytes"%self.buffer.tell())
        return res


class Camera(threading.Thread):
    "A class for acquiring continusly images..."

    def __init__(self, resolution=(3280, 2464), framerate=1, sensor_mode=3, 
                 avg_ev=21, avg_wb=31, histo_ev=None, wb_red=None, wb_blue=None, 
                 quit_event=None, queue=None, config_queue=None):
        """This thread handles the camera
        
        """
        threading.Thread.__init__(self, name="Camera")
        self.quit_event = quit_event or threading.Event()
        self._can_record = threading.Event()
        self._done_recording = threading.Event()
        self._done_recording.set()
        self._can_record.set()
        self.queue = queue or Queue()
        self.config_queue = config_queue or Queue()
        self.avg_ev = avg_ev
        self.avg_wb = avg_wb
        self.histo_ev = histo_ev or []
        self.wb_red = wb_red or []
        self.wb_blue = wb_blue or []
        raw_size = (((resolution[0]+31)& ~(31))*((resolution[1]+15)& ~(15))*3//2)
        self.stream = StreamingOutput(raw_size)
        self.camera = PiCamera(resolution=resolution, framerate=framerate, sensor_mode=sensor_mode)

    def __del__(self):
        self.camera = self.stream = None
    
    def quit(self):
        "quit the main loop and end the thread"
        self._quit.set()

    def pause(self):
        "pause the recording, wait for the current value to be acquired"
        self._can_record.clear()
        self._done_recording.wait()
    
    def resume(self):
        "resume the recording"
        self._can_record.set()

    
    def get_config(self):
        config = OrderedDict(("resolution", tuple(self.camera.resolution)),
                             ("framerate", float(self.camera.framerate)),
                             ("sensor_mode", self.camera.sensor_mode),
                             ("avg_ev", self.avg_ev),
                             ("avg_wb", self.avg_wb),
                             ("hist_ev", self.histo_ev),
                             ("wb_red", self.wb_red),
                             ("wb_blue", self.wb_blue))
        return config

    def set_config(self, dico):
        self.camera.resolution = dico.get("resolution", self.camera.resolution)
        self.camera.framerate = dico.get("framerate", self.camera.framerate)
        self.camera.sensor_mode = dico.get("sensor_mode", self.camera.sensor_mode)
        self.wb_red = dico.get("wb_red", self.wb_red)
        self.wb_blue = dico.get("wb_blue", self.wb_blue)
        self.histo_ev = dico.get("histo_ev", self.histo_ev)
        self.avg_ev = dico.get("avg_ev", self.avg_ev)
        self.avg_wb = dico.get("avg_wb", self.avg_wb)

    def get_metadata(self):
        metadata = {#"iso": float(self.camera.ISO),
                    "analog_gain": float(self.camera.analog_gain),
                    "awb_gains": [float(i) for i in self.camera.awb_gains],
                    "digital_gain": float(self.camera.digital_gain),
                    "exposure_compensation": float(self.camera.exposure_compensation),
                    "exposure_speed": float(self.camera.exposure_speed),
                    "framerate": float(self.camera.framerate),
                    "revision": self.camera.revision,
                    "shutter_speed": float(self.camera.shutter_speed),
                    "aperture": lens.aperture,
                    "resolution": self.camera.resolution}
        if metadata['revision'] == "imx219":
            metadata['iso'] = 54.347826086956516 * metadata["analog_gain"] * metadata["digital_gain"]
        else:
            metadata['iso'] = 100.0 * metadata["analog_gain"] * metadata["digital_gain"]
        return metadata


    def run(self):
        "main thread activity"
        self.camera.awb_mode = "off"
        self.camera.exposure_mode = "off"
        self.update_expo()
        
        self._done_recording.clear()
        for foo in self.camera.capture_continuous(self.stream, format='yuv'):
            self._done_recording.set()
            if self.stream.frame is not None:
                frame = self.stream.frame
                logger.info("Frame acquired: %s", frame.index)
                frame.camera_meta = self.get_metadata()
                self.queue.put(frame)
            else:
                logger.info("No frame acquired")
            if self.quit_event.is_set():
                break
            # update the camera settings if needed:
            if not self.config_queue.empty():
                while not self.config_queue.empty():
                    evrb = self.config_queue.get()
                    if evrb.red:
                        self.wb_red.append(evrb.red)
                        self.wb_blue.append(evrb.blue)
                    self.histo_ev.append(evrb.ev)
                self.update_expo()    
            self._can_record.wait()
            self._done_recording.clear()
        self.camera.close()

    def update_expo(self):
        """This method updates the white balance, exposure time and gain
        according to the history
        """ 
        if len(self.wb_red) * len(self.wb_blue) * len(self.histo_ev) == 0:
            return
        if len(self.wb_red) > self.avg_wb:
            self.wb_red = self.red_gains[-self.avg_wb:]
            self.wb_blue = self.wb_blue[-self.avg_wb:]
        if len(self.histo_ev) > self.avg_ev:
            self.histo_ev = self.histo_ev[-self.avg_ev:]
        self.camera.awb_gains = (savgol(self.red_gains),
                                 savgol(self.blue_gains))
        ev = savgol(self.histo_ev)
        speed = lens.calc_speed(ev)
        if speed > self.framerate:
            camera.shutter_speed = int(1000000. / self.framerate / speed)
            camera.iso = 100
        elif speed > self.framerate * 2:
            camera.shutter_speed = int(2000000. / self.framerate / speed)
            camera.iso = 200
        elif speed > self.framerate * 4:
            camera.shutter_speed = int(4000000. / self.framerate / speed)
            camera.iso = 400
        else:
            camera.shutter_speed = min(int(8000000. / self.framerate / speed), 1000000)
            camera.iso = 800       
        #TODO: how to change framerate ? maybe start with low
        

class Saver(threading.Thread):
    "This thread is in charge of saving the frames arriving from the queue on the disk"
    def __init__(self, directory=".", queue=None, quit_event=None):
        threading.Thread(self, name="Saver")
        self.queue = queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        self.directory = os.path.abspath(directory)
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    def run(self):
        while not self.quit_event.is_set():
            frame = self.queue.get()
            if frame:
                name = os.path.join(self.directory, frame.get_date_time()+".jpg")
                Image.fromarray(frame.rgb).save(name, quality=90, optimize=True, progressive=True)
                exif = pyexiv2.ImageMetadata(name)
                exif.read()
                exposure_speed = frame.camera_meta.get("exposure_speed", 1)
                speed = Fraction(1000000, int(exposure_speed))
                iso = int(100 * frame.camera_meta.get("digital_gain", 1) * frame.camera_meta.get("analog_gain", 1))
                exif["Exif.Photo.FNumber"] = Fraction(int(frame.camera_meta.get("aperture") * 100), 100)
                exif["Exif.Photo.ExposureTime"] = speed
                exif["Exif.Photo.ISOSpeedRatings"] = iso
                comments = OrderedDict()
                if frame.position:
                    comments["pan"] = frame.position.pan
                    comments["tilt"] = frame.position.tilt
                if frame.gravity:
                    comments["gx"] = frame.gravity.x
                    comments["gy"] = frame.gravity.y
                    comments["gz"] = frame.gravity.z
                exif.comment = json.dumps(comments)
                exif.write(preserve_timestamps=True)
            self.queue.task_done()


class Analyzer(threading.Thread):
    "This thread is in charge of analyzing the image and suggesting new exposure value and white balance"
    def __init__(self, queue=None, quit_event=None):
        threading.Thread(self, name="Saver")
        self.queue = queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        self.history = []
        self.max_size = 100
        
        i = numpy.arange(40)
        j = 0.5 ** (0.25 * i)
        k = ((j + 0.099) / 1.099) ** (1 / 0.45) * (235 - 16) + 16
        m2 = j < 0.018
        k[m2] = (235-16) / 4.5 * j[m2] + 16
        kr = numpy.round(k).astype(int)
        self.ukr = numpy.concatenate(([0], numpy.sort(numpy.unique(kr)), [256]))
        start = -0.25*(self.ukr.size-1)+0.5
        self.delta_expo = numpy.arange(start, 0.5, 0.25)
        self.g19_2 = gaussian(19, 2)
        self.g19_2 /= self.g19_2.sum()
        

    def run(self):
        """This executed in a thread"""
        target_rgb = 5e-4 # pixels at 99.5% should be white
        while not self.quit_event.is_set():
            frame = self.queue.get()
            t0 = time.time()
            ev = lens.calc_EV(1000000/frame.camera_meta.get("exposure_speed", 1))
            yprime = frame.yuv[:, :, 0]
            cnt = numpy.bincount(yprime, minlength=256)
            cs = numpy.zeros(257, int)
            cs[1:] = numpy.cumsum(cnt)
            dev = cs[self.ukr[1:]]-cs[self.ukr[:-1]]
            cnv = convolve(dev, self.g19_2, mode="same")
            idx = cnv.argmax()
            dev = self.delta_expo[idx]
            hess = (cnv[idx + 1] + cnv[idx - 1] - 2 * cnv[idx])
            if hess:
                 dev -= 0.5 * (cnv[idx + 1] - cnv[idx - 1]) / hess
            csr = numpy.zeros(257, int)
            csg = numpy.zeros(257, int)
            csb = numpy.zeros(257, int)
            pos = frame.rgb.shape[0] * frame.rgb.shape[1] * (1.0 - target_rgb)
            csr[1:] = numpy.cumsum(numpy.bincount(frame.rgb[:,:,0], minlength=256))
            csg[1:] = numpy.cumsum(numpy.bincount(frame.rgb[:,:,1], minlength=256))
            csb[1:] = numpy.cumsum(numpy.bincount(frame.rgb[:,:,2], minlength=256))
            pos_r = numpy.where
            
            
            now = time.time()
            logger.info("Analysis of frame #%i took: %.3fs, delay since acquisition: %.3fs", frame.index, now-t0, now-frame.timestamp)

