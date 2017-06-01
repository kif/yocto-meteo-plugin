#!/usr/bin/env python
from __future__ import division, print_function
import signal
from picamera import PiCamera

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
import math
from fractions import Fraction
import json
from collections import namedtuple, OrderedDict
from argparse import ArgumentParser
import servo
from accelero import Accelerometer
from exposition import Exposition
from camera import Camera, Saver, Analyzer, Frame
import logging
logger = logging.getLogger("camera")


Position = namedtuple("Position", ("pan", "tilt"))


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
        self.camera.pause()
        self.servo_tilt.move(tilt)
        self.servo_pan.move(pan)
        time.sleep(self.delay_off)
        self.servo_tilt.off()
        self.servo_pan.off()
        self.accelero.resume()
        self.camera.resume()
 
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
