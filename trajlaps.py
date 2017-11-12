#!/usr/bin/env python
from __future__ import division, print_function

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("trajlaps")


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
from collections import namedtuple, OrderedDict, deque
from argparse import ArgumentParser
import servo
from accelero import Accelerometer
from exposure import lens
from camera import Camera, Saver, Analyzer, Frame


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
    def pause(self, wait=True):
        pass
    def resume(self):
        pass

class Trajectory(object):
    def __init__(self, delay_off=1, accelero=None, camera=None, config=None, json_file=None):
        self.start_time = time.time()
        self.accelero = accelero or DummyAccelero()
        self.camera = camera or DummyAccelero()
        self.delay_off = delay_off
        self.config = config or []
        self.servo_tilt = servo.tilt
        self.servo_pan = servo.pan
        self.default_pos = Position(0, 0)
        if json_file:
            self.load_config(json_file)
        self.goto_pos(self.default_pos)
        

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
        logger.info("Config set to :%s ", self.config)

    def goto(self, when):
        """move the camera to the position it need to be at a given timestamp"""
        pos = self.calc_pos(when)
        logger.info(pos)
        threading.Thread(target=self.goto_pos,args=(pos,)).start()
        return pos

    def goto_pos(self, pos):
        """Move the camera to the given position
        Operates usually in a separated thread
		"""
        pan, tilt = pos
        self.camera.pause(wait=False)
        self.accelero.pause()
        self.camera.pause()
        self.servo_tilt.move(tilt)
        self.servo_pan.move(pan)
        time.sleep(self.delay_off)
        #self.servo_tilt.off()
        #self.servo_pan.off()
        self.accelero.resume()
        self.camera.resume()
        res = OrderedDict([("pan", self.servo_pan.get_config()),
                           ("tilt", self.servo_tilt.get_config())])
        return res
 
    def calc_pos(self, when):
        """Calculate the position it need to be at a given timestamp"""
        next_pos = last_pos = self.default_pos
        last_timestamp = remaining = when
        if remaining <= 0:
            return last_pos
        #print(self.config)
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
            #we ran out of points: stay on the last
            return next_pos
        delta = last_timestamp - remaining
        delta_p = next_pos.pan - last_pos.pan
        delta_t = next_pos.tilt - last_pos.tilt
        adv = last_timestamp / delta
        tilt = last_pos.tilt + adv * delta_t
        pan = last_pos.pan + adv * delta_p
        #print(last_timestamp, remaining, delta, adv)
        return Position(pan,tilt)


class TimeLaps(threading.Thread):
    def __init__(self, resolution=(3280, 2464), framerate=1, delay=20, 
                 folder=".", avg_awb=200, avg_ev=25, config_file="parameters.json"):
        threading.Thread.__init__(self, name="TimeLaps")
        self.storage = {}
        self.storage_maxlen = 10
        self.pool_of_analyzers = []
        self.pool_size_analyzer = 2
        self.pool_of_savers = []
        self.pool_size_savers = 2
        self.do_analysis = False#True
        self.camera_queue = Queue()
        self.analysis_queue = Queue()
        self.saving_queue = Queue()
        self.config_queue = Queue()
        self.quit_event = threading.Event()
        self.delay = delay
        self.folder = os.path.abspath(folder)
        self.avg_wb = avg_awb # time laps over which average gains
        self.avg_ev = avg_ev # time laps over which average speed
        self.config_file = os.path.abspath(config_file)
        self.position = Position(0,0)
        self.start_time = self.last_img = time.time()
        self.next_img = self.last_img + 2 * self.delay
        signal.signal(signal.SIGINT, self.quit)
        self.accelero = Accelerometer() 
        self.accelero.start()
        self.folder = folder
        self.servo_status = None
        
        self.camera = Camera(resolution=resolution,                              
                             framerate = framerate,
                             avg_ev=avg_ev, 
                             avg_wb=avg_awb, 
                             histo_ev=None, 
                             wb_red=None, 
                             wb_blue=None,
                             queue=self.camera_queue,
                             config_queue=self.config_queue,
                             quit_event=self.quit_event,
                             )
        self.camera.warm_up()      
        self.trajectory = Trajectory(accelero=self.accelero, camera=self.camera)
        self.load_config(config_file)
        
        for i in range(self.pool_size_savers):
            saver = Saver(folder=self.folder,
                          queue=self.saving_queue, 
                          quit_event=self.quit_event)
            saver.start()
            self.pool_of_savers.append(saver)
            
        for i in range(self.pool_size_analyzer):
            analyzer = Analyzer(frame_queue=self.analysis_queue, 
                                 config_queue=self.config_queue, 
                                 quit_event=self.quit_event)
            analyzer.start()
            self.pool_of_analyzers.append(analyzer)
        self.position = self.trajectory.goto(self.delay)  
        self.camera.start()
    
    def __del__(self):
        self.quit_event.set()
        self.camera = None
        self.trajectory = None
        self.accelero = None

    def quit(self, *arg, **kwarg):
        """called with SIGINT: clean up and quit gracefully"""
        self.quit_event.set()

    def load_config(self, config_file=None):
        if config_file is None:
            config_file=self.config_file
            
        if os.path.isfile(config_file):
            with open(self.config_file) as jsonfile:
                dico = json.load(jsonfile)
            #print(dico)
            if "trajectory" in dico:
				self.trajectory.set_config(dico["trajectory"]) 
            if "camera" in dico:
                self.camera.set_config(dico["camera"])
            self.delay = dico.get("delay", self.delay)
            self.folder = dico.get("folder", self.folder)
            self.do_analysis = dico.get("do_analysis", self.do_analysis)
            self.camera.set_analysis(self.do_analysis)
            
    def save_config(self, index=None):
        if index is not None:
            fname = os.path.splitext(self.config_file)
            fname = fname[0] + "_%05i"%index + fname[1]
        else:
            fname = self.config_file
        logger.info("Save config to %s", fname)
        camera_config = self.camera.get_config()
        dico = OrderedDict([                           
                            ("delay", self.delay),
                            ("folder", self.folder),
                            ("do_analysis", self.do_analysis),
                            ("trajectory", self.trajectory.config),
                            ("camera", camera_config),
                            ])
        with open(fname,"w") as jsonfile:
            jsonfile.write(json.dumps(dico, indent=4))

    def run(self):
        "Actually does the timelaps"
        self.camera.set_analysis(self.do_analysis)
        while not self.quit_event.is_set():
            frame = self.camera_queue.get()
            frame.position = self.position
            frame.servo_status = self.servo_status
            if self.do_analysis:
                self.analysis_queue.put(frame)
            if self.position not in self.storage:
                self.storage[self.position] = deque(maxlen=self.storage_maxlen) 
            self.storage[self.position].append(frame) 
            if time.time() >= tl.next_img:
                frame.gravity = self.accelero.get()
                self.saving_queue.put(self.storage.pop(self.position))
                self.next_img = frame.timestamp + self.delay
                next_pos = self.trajectory.calc_pos(self.next_img - self.start_time)
                if next_pos != self.position:
                    self.servo_status = self.trajectory.goto_pos(next_pos)
                    self.position = next_pos
            self.camera_queue.task_done()
            logger.info("Frame #%04i. Length of queues: camera %i, analysis %i saving %i config %i", 
                            frame.index,
                            self.camera_queue.qsize(), 
                            self.analysis_queue.qsize(),
                            self.saving_queue.qsize(),
                            self.config_queue.qsize()
                            )
            if frame.index % 100 == 0:
                self.save_config(frame.index)
                               

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
    parser.add_argument("-d", "--debug", help="debug", default=False, action="store_true")
    
    args = parser.parse_args()
    if args.debug:
        logging.root.setLevel(logging.DEBUG)
    tl = TimeLaps(config_file=args.json, framerate=0.1)
    tl.run()
