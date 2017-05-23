#!/usr/bin/env python
from __future__ import division, print_function
import signal
from picamera import PiCamera
from PIL import Image
import time 
import numpy
import threading
try:
    from Queue import Queue
except:
    from queue import Queue
import datetime
import os
import sys
import io
import math
from fractions import Fraction
import json
import pyexiv2
from collections import namedtuple, OrderedDict
from argparse import ArgumentParser
import servo
from accelero import Accelerometer
from exposition import Exposition
from scipy.signal import convolve, gaussian, savgol_coeffs


Position = namedtuple("Position", ("pan", "tilt"))
ExpoRedBlue = namedtuple("ExpoRedBlue", ("ev", "red", "blue"))

trajectory={
"delay": 20,
"avg_wb":200,
"avg_ev":6,
"trajectory": [
 {"tilt":30,
  "pan":-50,
  "stay": 10,
  "move": 10},
 {"tilt":30,
  "pan":70,
  "stay": 10,
  "move": 21000},
 {"tilt":30,
  "pan":-50,
  "stay": 10,
  "move": 1000},
 ],
"histo_ev": [9,9,9,9],
"wb_red": [1.50390625,1.50390625,1.50390625,1.50390625],
"wb_blue": [1.56640625,1.56640625,1.56640625,1.56640625]
}


class SavGol(object):
    "Class for Savitsky-Golay filtering"
    def __init__(self, order=2):
        "select the order of the filter"
        self.order = order
        self.cache = {} #len, filter

    def __call__(lst):
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

    
class DummyAccelero(object):
    """Dummy accelerometer"""
    def pause(self):
        pass
    def resume(self):
        pass

class Trajectory(object):
    def __init__(self, delay_off=0.5, accelero=None, config=None, json_file=None):
        self.start_time = time.time()
        self.accelero = accelero or DummyAccelero()
        self.delay_off = delay_off
        self.config = config or []
        self.servo_tilt = servo.tilt
        self.servo_pan = servo.pan
        self.default_pos = Position(0, 0)
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
           self.set_config(config["trajectory"])
        else:
           self.set_config(config)

    def get_config(self):
        return self.config

    def set_config(self, config):
        self.config = list(config)

    def goto(self, when):
        """move the camera to the position it need to be at a given timestamp"""
        pos = self.calc_pos(when)
        threading.Thread(target=self.goto_pos,args=(pos,)).start()
        return pos

    def goto_pos(self, pos):
        """Move the camera to the given position
        Operates usually in a separated thread
		"""
        pan, tilt = pos
        self.accelero.pause()
        self.servo_tilt.move(tilt)
        self.servo_pan.move(pan)
        time.sleep(self.delay_off)
        self.servo_tilt.off()
        self.servo_pan.off()
        self.accelero.resume()
 
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


class TimeLaps(object):
    def __init__(self, resolution=(2592, 1952), framerate=1, delay=360, 
                 folder=".", avg_awb=200, avg_ev=25, config_file="parameters.json"):
        self.camera_queue = Queue()
        self.analysis_queue = Queue()
        self.saving_queue = Queue()
        self.quit_event = Event()
        self.delay = delay
        self.folder = os.path.abspath(folder)
        self.avg_wb = avg_awb # time laps over which average gains
        self.avg_ev = avg_ev # time laps over which average speed
        self.config_file = os.path.abspath(config_file)
        self.position = Position(0,0)
        self.lens = Exposition()
        self.start_time = self.last_img = time.time()
        self.next_img = self.last_img + 2 * self.delay
        signal.signal(signal.SIGINT, self.quit)
        self.accelero = Accelerometer() 
        self.accelero.start()
        self.trajectory = Trajectory(accelero=self.accelero)
        self.camera = Camera(resolution=resolution,                              
                             queue=self.camera_queue,
                             framerate = framerate,
                             avg_ev=avg_ev, 
                             avg_wb=avg_awb, 
                             histo_ev=None, 
                             wb_red=None, 
                             wb_blue=None 
                             quit_event=self.quit_event, 
                             queue=self.processing_queue,
                             lens = self.lens
                             )
        self.load_config() 

        self.camera.start()
        self.position = self.trajectory.goto(self.delay)        
    
    def __del__(self):
        self.quit_event.set()
        self.camera = None
        self.trajectory = None
        self.accelero = None

    def quit(self, *arg, **kwarg):
        """called with SIGINT: clean up and quit gracefully"""
        self.quit_event.set()

    def load_config(self):
        if os.path.isfile(self.config_file):
            with open(self.config_file) as jsonfile:
                dico = json.load(jsonfile)
            if "trajectory" in dico:
				self.trjectory = dico["trajectory"] 
            if "camera" in dico:
                self.camera.set_config(dico["camera"])
            self.delay = dico.get("delay", self.delay)
            
    def save_config(self):
        camera_config = self.camera.get_config()
        dico = OrderedDict([                           
                            ("delay", self.delay),
                            ("trajectory", self.trajectory.config),
                            ("camera", camera_config),
                            ])
        with open(self.config_file,"w") as jsonfile:
            jsonfile.write(json.dumps(dico, indent=4))

    def capture(self):
        """Take a picture with the camera, if there is enough light
        """
        if not self.is_valid:
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
        threading.Thread(target=self.save,args=(self.raw.array.copy(), fname, ns, iso)).start()
        #self.save(self.raw.array, fname, ns, iso)
        self.raw.truncate(0)
        self.camera.awb_mode = "auto"
        self.camera.iso = 0
        self.camera.shutter_speed = 0
        self.position = self.trajectory.goto(self.next_img - self.start_time)

    def measure(self):   
        self.camera.capture(self.raw, 'rgb')
        self.raw.truncate(0)
        r,b = self.camera.awb_gains
        print("Measured exposure_speed: %f analog_gain: %f digital_gain: %f"%(self.camera.exposure_speed, self.camera.analog_gain, self.camera.digital_gain))
        self.last_speed = 1.0e6 / self.camera.exposure_speed
        self.last_gain = 1.0 * self.camera.analog_gain * self.camera.digital_gain
        if not self.is_valid:
            return
        r = 1.0 * r
        b = 1.0 * b
        if r == 0.0: 
            r = 1.0
        if b == 0.0:
            b = 1.0

        self.last_wb = (r, b)
        ls = self.last_speed / self.last_gain
        self.red_gains.append(r)
        self.blue_gains.append(b)
        self.speeds.append(math.log(ls, 2.0))
        if len(self.red_gains) > self.avg_wb:
            self.red_gains = self.red_gains[-self.avg_wb:]
            self.blue_gains = self.blue_gains[-self.avg_wb:]
        if len(self.speeds) > self.avg_ev:
           self.speeds = self.speeds[-self.avg_ev:]
        if len(self.speeds) < len(SG):
            self.speeds += [self.speeds[-1]] * (len(SG) - len(self.speeds))
        rg = sum(self.red_gains)/len(self.red_gains)
        bg = sum(self.blue_gains)/len(self.blue_gains)
        mgspeed = sum(i*j for i,j in zip(SG, self.speeds[-len(SG):]))
        ns = 2.0 ** (mgspeed)
        #ns = 2.0 ** (sum(self.speeds)/len(self.speeds))
        print("len %s/%s, \t S %.3f/iso%.0f -> %.3f \t R %.3f-> %.3f \t B %.3f -> %.3f"%(len(self.red_gains),len(self.speeds), self.last_speed, 100.*self.last_gain, ns, r, rg, b, bg))
        self.next_speed = ns
        self.next_wb = rg,bg
        self.save_config()

    @property
    def is_valid(self):
       return self.last_speed/self.last_gain >=0.4 
    

class Frame(object):
    """This class holds one image"""
    INDEX = 0 
    semclass = threading.Semaphore()
            # YUV conversion matrix from ITU-R BT.601 version (SDTV)
            #                  Y       U       V
    YUV2RGB = numpyp.array([[1.164,  0.000,  1.596],                         # R
                           [1.164, -0.392, -0.813],                          # G
                           [1.164,  2.017,  0.000]], dtype=numpy.float32)    # B

    def __init__(self, data):
        "Constructor"
        self.timestamp = time.time()
        with self.semclass:
            self.index = self.INDEX
            self.__class__.INDEX+=1
        self.data = data
        self.camera_meta = {}
        self.gravity = None
        self.position = None
        self.sem = threading.Semaphore()
        self._yuv = None
        self._rgb = None

    def save(self):
        "Save the data as YUV raw data"
        with open("dump_%s.yuv"%self.timestamp, "w") as f:
            f.write(self.data)
        print("Saved frame %i %s"%(self.index, self.timestamp))

    def get_date_time(self):
        return time.strftime("%Y-%m-%d-%Hh%Mm%Ss", time.localtime(self.timestamp)) 

    @property
    def yuv(self):
        """Retrieve the YUV array"""
        if self._yuv is None:
            with self.sem:
                if self._yuv is None:
                    width, height = self.camera_meta.get("resolution", (640,480))
                    fwidth = (width + 31) & ~(31)
                    fheight = (height + 15) & ~ (15)
                    ylen = fwidth * fheight
                    a = np.frombuffer(data, dtype=np.uint8)
                    Y = a[:y_len]
                    U = a[y_len:-uv_len]
                    V = a[-uv_len:]
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
            if self._yub is None:
                YUV = numpy.asarray(self.yuv, dtype=numpy.float32)
            with self.sem:
                if self._rgb is None:
                    YUV[:, :, 0]  = YUV[:, :, 0]  - 16.0  # Offset Y by 16
                    YUV[:, :, 1:] = YUV[:, :, 1:] - 128.0 # Offset UV by 128
                    # Calculate the dot product with the matrix to produce RGB output,
                    # clamp the results to byte range and convert to bytes
                    self._rgb = YUV.dot(self.YUV2RGB.T).clip(0, 255).astype(numpy.uint8)
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

    def __init__(self, resolution=(3280, 2464), framerate=1, image_mode=3, 
                 avg_ev=21, avg_wb=31, histo_ev=None, wb_red=None, wb_blue=None 
                 quit_event=None, queue=None, config_queue=None, lens=None):
        """This thread handles the camera
        
        """
        threading.Thread.__init__(self, name="Camera")
        self.quit_event = quit_event or threading.Signal()
        self.queue = queue or Queue()
        self.config_queue = config_queue or Queue()
        self.filter = SavGol(order=2)
        self.lens = lens or Exposition()
        self.avg_ev = avg_ev
        self.avg_bw = avg_wb
        self.histo_ev = histo_ev or []
        self.wb_red = wb_red or []
        self.wb_blue = wb.blue or []
        raw_size = (((resolution[0]+31)& ~(31))*((resolution[1]+15)& ~(15))*3//2)
        self.stream = StreamingOutput(raw_size)
        self.camera = picamera.PiCamera(resolution=resolution, framerate=framerate, image_mode=sensor_mode)
        self.camera.awb_mode = "off"
        self.camera.exposure_mode = "off"
        self.update_expo()

    def __del__(self):
        self.camera = self.stream = None
    
    def get_config(self):
        config = OrderedDict(("resolution", tuple(self.camera.resolution)),
                             ("framerate", float(self.camera.framerate)),
                             ("sensor_mode", self.camera.sensor_mode),
                             ("avg_ev", self.avg_ev),
                             ("avg_wb", self.avg_wb),
                             ("hist_ev", self.hist_ev),
                             ("wb_red", self.wb_red),
                             ("wb_blue". self.wb.blue))
        return config

    def set_config(self, dico):
        self.camera.resolution = dico.get("resolution", self.camera.resolution)
        self.camera.framerate = dico.get("framerate", self.camera.framerate)
        self.camera.sensor_mode = dico.get("sensor_mode", self.camera.sensor_mode)
        self.red_gains = dico.get("wb_red", self.red_gains)
        self.blue_gains = dico.get("wb_blue", self.blue_gains)
        self.histo_ev = dico.get("histo_ev", self.histo_ev)
        self.avg_ev = dico.get("avg_ev", self.avg_ev)
        self.avg_wb = dico.get("avg_wb", self.avg_wb)

    def get_metadata(self)
        metadata = {"iso": float(camera.ISO),
                    "analog_gain": float(camera.analog_gain),
                    "awb_gains": [float(i) for i in camera.awb_gains],
                    "digital_gain": float(camera.digital_gain),
                    "exposure_compensation": float(camera.exposure_compensation),
                    "exposure_speed": float(camera.exposure_speed),
                    "framerate": float(camera.framerate),
                    "revision": camera.revision,
                    "shutter_speed": float(camera.shutter_speed)
                    "aperture": 2.8,
                    "resolution": camera.resolution}
        return metadata


    def run(self):
        "main thread activity"
        for foo in self.camera.capture_continuous(self.stream, format='yuv'):
            if self.stream.frame is not None:
                frame = stream.frame
                frame.camera_meta = metadata
                self.queue.put(frame)
            if self.quit_event.is_set():
                break
            # update the camera settings if needed:
            if not self.config_queue.empty():
                while not self.config_queue.empty():
                    evrb = self.config_queue.get()
                    if evrb.red:
                        self.red_gains.append(evrb.red)
                        self.blue_gains.append(evrb.blue)
                    self.histo_ev.append(evrb.ev)
                self.update_expo()    

        self.camera.close()

    def update_expo(self):
        """This method updates the white balance, exposure time and gain
        according to the history
        """ 
        if len(self.red_gains) > self.avg_wb:
            self.red_gains = self.red_gains[-self.avg_wb:]
            self.blue_gains = self.blue_gains[-self.avg_wb:]
        if len(self.histo_ev) > self.avg_ev:
            self.histo_ev = self.histo_ev[-self.avg_ev:]
        self.camera.awb_gains = (self.filter(self.red_gains),
                                 self.filter(self.blue_gains))
        ev = self.filter(self.histo_ev)
        speed = self.lens.calc_speed(ev)
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
        

class Saver(threading.thread):
    "This thread is in charge of saving the frames arriving from the queue on the disk"
    def __init__(self, directory=".", queue=None, quit_event=None):
        threading.Thread(self, name="Saver")
        self.queue = queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        self.directory = os.path.abspath(directory)
        if not os.path.exists(self.directory):
            os.makedirs(self.directory

    def run(self):
        while not self.quit_event.is_set():
            frame = self.queue.get()
            if frame:
                name = os.path.join(self.directory, frame.get_date_time()+".jpg")
                Image.fromarray(frame.rgb).save(name, quality=90, optimize=True, progressive=True)
                exif = pyexiv2.ImageMetadata(name)
                exif.read()
                speed = Fraction(1000000, int(exposure_speed))
                iso = int(100 * frame.camera_meta.get("digital_gain", 1) * frame.camera_meta.get("analog_gain",1))
                exif["Exif.Photo.FNumber"] = Fraction(int(frame.camera_meta.get("aperture")*100),100)
                exif["Exif.Photo.ExposureTime"] = speed
                exif["Exif.Photo.ISOSpeedRatings"] = iso
                comments = OrderedDict()
                if frame.position:
                    comments["pan"] = frame.position.pan
                    comments["tilt"] =  frame.position.tilt
                if frame.gravity:
                    comments["gx"] = frame.gravity.x
                    comments["gy"] = frame.gravity.y
                    comments["gz"] = frame.gravity.z
                exif.comment = json.dumps(comments)
                exif.write(preserve_timestamps=True)
            self.queue.task_done()


class Analyzer(threading.thread):
    "This thread is in charge of analyzing the image and suggesting new exposure value and white balance"
    def __init__(self, directory=".", queue=None, quit_event=None, lens=None):
        threading.Thread(self, name="Saver")
        self.queue = queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        self.lens = lens or Exposition()
        self.history = []
        self.max_size = 100
        
        i = numpy.arange(40)
        j= 0.5 ** (0.25 * i)
        k = ((j + 0.099) / 1.099) ** (1 / 0.45) * (235 - 16) + 16
        m2 = j < 0.018
        k[m2] = (235-16) / 4.5 * j[m2] + 16
        kr = numpy.round(k).astype(int)
        ukr = numpy.concatenate(([0],numpy.sort(numpy.unique(kr)),[256]))
        start = -0.25*(ukr.size-1)+0.5
        self.delta_expo = numpy.arange(start,0.5, 0.25)
        

        self.g19_2 = gaussian(19, 2)
        self.g19_2 /= self.g19_2.sum()

    def run(self):
        """This executed in a thread"""
        while not self.quit_event.is_set():
            frame = self.queue.get()
            ev = self.lens.calc_EV(1000000/frame.camera_meta.get("exposure_speed",1)
            yprime = frame.yuv[:,:,0]
            cnt = numpy.bincount(yprime, minlength=256)
            cs = numpy.zeros(257, int)
            cs[1:] = numpy.cumsum(cnt)
            dev = cs[ukr[1:]]-cs[ukr[:-1]]
            cnv = convolve(dev, g19_2, mode="same")
            idx = cnv.argmax()
            dev = self.delta_expo[idx]-0.5*(cnv[idx+1]-cnv[idx-1])/(cnv[idx+1]+cnv[idx-1]-2*cnv[idx])


if __name__ == "__main__":
    try: 
        from rfoo.utils import rconsole
    except:
        pass
    else:
        rconsole.spawn_server()

    parser = ArgumentParser("trajlaps", 
                            description="TimeLaps over a trajectory")
    parser.add_argument("-j", "--json", help="config file")
    args = parser.parse_args()
    tl = TimeLaps(resolution=(config_file=args.json)
    print("Warmup ... %s seconds"%(2*tl.delay))
    while not tl.quit_event.is_set():
        frame = tl.camera_queue.get()
        if time.time() < tl.next_img:        
            tl.analysis_queue.put(frame)
        else:
            frame.position = tl.position
            frame.gravity = tl.accelero.get()
            tl.saving_queue.put(frame)
            tl.next_img = frame.timestamp + tl.delay
            tl.position = tl.trajectory.goto(tl.next_img - tl.start_time)
        tl.camera_queue.task_done()
