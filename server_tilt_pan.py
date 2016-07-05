#!/usr/bin/env python
#coding: utf-8

from __future__ import division, print_function
from picamera import PiCamera
from PIL import Image
import Adafruit_PCA9685
import os
import glob
import bottle
from argparse import ArgumentParser
import datetime
from collections import namedtuple
import json

Position = namedtuple("Position", ("pan", "tilt"))


bottle.debug(True)
root = os.path.dirname(os.path.abspath(__file__))

class Server(object):
    page = """
<html>
<header>
<title> pan= {pan} tilt= {tilt}</title>
</header>
<body>
<center>
<p>
Pan: {pan} Tilt: {tilt}
</p><p> 
<a href="reload" title="re-take image">Capture</a>
<a href="save" title="add position to trajectory">Save pos</a>
</p><p>
<a href="pan_min" title="pan -90"> |&lt </a>
<a href="pan_-10" title="pan -=10"> &lt&lt </a>
<a href="pan_-01" title="pan -=01"> &lt </a>
<a href="pan_0" title="pan =0"> pan center </a>
<a href="pan_+1" title="pan +=01"> &gt </a>
<a href="pan_+10" title="pan +=10"> &gt&gt </a>
<a href="pan_max" title="pan +90"> &gt| </a>
</p><p>
<a href="tilt_min" title="tilt = -50"> |&lt </a>
<a href="tilt_-10" title="tilt -= 10"> &lt&lt </a>
<a href="tilt_-1" title="tilt -= 01"> &lt </a>
<a href="tilt_0" title="tilt = 0"> Tilt center </a>
<a href="tilt_+1" title="tilt += 1"> &gt </a>
<a href="tilt_+10" title="tilt += 10"> &gt&gt </a>
<a href="tilt_max" title="tilt +90"> &gt| </a>
</p>
<p><img src="{image}" title="{date_time}"/></p>
</center>
</body>
</html>
    """
    def __init__(self, img_dir="images", ip="0.0.0.0", port=80, timer=30):
        self.img_dir = img_dir
        if not os.path.exists(self.img_dir):
            os.mkdir(self.img_dir)
        self.trajectory = []
        self.traj_file = datetime.datetime.now().strftime("%Y-%m-%d-%Hh%Mm%Ss.json")
        self.ip = ip
        self.port = port
        self.bottle = bottle.Bottle()
        self.setup_routes()
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.servo_id_tilt = 15
        self.servo_id_pan = 14
        self.default_pos = Position(0, 0)
        self.current_pos = self.default_pos
        self.cam = None
        self.resolution = (1296, 972)
        self.last_image = None
        self.servo_min = 150 # out of 4096
        self.servo_max = 600 # out of 4096
        self.setup_cam()

    def setup_routes(self):
        self.bottle.route('/images/:filename', callback=self.server_static)
        self.bottle.route('/', callback=self.index)
        self.bottle.route('/pan_min', callback=self.pan_min)
        self.bottle.route('/pan_-10', callback=self.pan_sub10)
        self.bottle.route('/pan_-01', callback=self.pan_sub1)
        self.bottle.route('/pan_0' , callback=self.pan_center)
        self.bottle.route('/pan_+1', callback=self.pan_add1)
        self.bottle.route('/pan_+10', callback=self.pan_add10)
        self.bottle.route('/pan_max', callback=self.pan_max)
        self.bottle.route('/tilt_min', callback=self.tilt_min)
        self.bottle.route('/tilt_-10', callback=self.tilt_sub10)
        self.bottle.route('/tilt_-1', callback=self.tilt_sub1)
        self.bottle.route('/tilt_0', callback=self.tilt_center)
        self.bottle.route('/tilt_+1', callback=self.tilt_add1)
        self.bottle.route('/tilt_+10' , callback=self.tilt_add10)
        self.bottle.route('/tilt_max', callback=self.tilt_max)
        self.bottle.route('/reload', callback=self.move)
        self.bottle.route("/save", callback=self.save)

    def server_static(self, filename):
        return bottle.static_file(filename, root=os.path.join(root, self.img_dir))
    
    def index(self):
        return self.move()

    def start(self):
        """Start the serveur (does not return):::"""
        self.bottle.run(host=self.ip, port=self.port)

    def move(self, new_pos=None):
        new_pos = new_pos or self.current_pos
        if new_pos.tilt<-50:
            new_pos = Position(new_pos.pan, -50)
        elif new_pos.tilt>90:
            new_pos = Position(new_pos.pan, 90)
        if new_pos.pan <-90:
            new_pos = Position(-90, new_pos.tilt)
        elif new_pos.pan>90:
            new_pos = Position( 90, new_pos.tilt)
        self.goto_pos(new_pos)
        #TODO: take picture and update web page and return it
        dico={"tilt": new_pos.tilt,
              "pan": new_pos.pan,
              }
        dico["date_time"] = self.capture()
        dico["image"] = self.last_image
        webpage = self.page.format(**dico)
        self.current_pos = new_pos
        return webpage

    def goto_pos(self, pos):
        pan, tilt = pos
        self.pwm.set_pwm(self.servo_id_pan, 0, self.angle(pan))
        self.pwm.set_pwm(self.servo_id_tilt, 0, self.angle(tilt))

    def pan_min(self):
        pos = Position(-90, self.current_pos.tilt)
        return self.move(pos)

    def pan_max(self):
        pos = Position(+90, self.current_pos.tilt)
        return self.move(pos)
    
    def tilt_min(self):
        pos = Position(self.current_pos.pan, -50)
        return self.move(pos)

    def tilt_max(self):
        pos = Position(self.current_pos.pan, +90)
        return self.move(pos)

    def pan_sub1(self):
        pos = Position(self.current_pos.pan -1 , self.current_pos.tilt)
        return self.move(pos)

    def pan_add1(self):
        pos = Position(self.current_pos.pan + 1, self.current_pos.tilt)
        return self.move(pos)
    
    def tilt_sub1(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt -1)
        return self.move(pos)

    def tilt_add1(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt +1)
        return self.move(pos)

    def pan_sub10(self):
        pos = Position(self.current_pos.pan - 10, self.current_pos.tilt)
        return self.move(pos)

    def pan_add10(self):
        pos = Position(self.current_pos.pan + 10, self.current_pos.tilt)
        return self.move(pos)
    
    def tilt_sub10(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt - 10)
        return self.move(pos)

    def tilt_add10(self):
        pos = Position(self.current_pos.pan, self.current_pos.tilt + 10)
        return self.move(pos)

    def pan_center(self):
        pos = Position(0, self.current_pos.tilt)
        return self.move(pos)

    def tilt_center(self):
        pos = Position(self.current_pos.pan, 0)
        return self.move(pos)

    def setup_cam(self):
        self.cam = PiCamera()
        self.cam.resolution = self.resolution
        self.capture()
 
    def capture(self):
        now = datetime.datetime.now().strftime("%Y-%m-%d-%Hh%Mm%Ss")
        fname = "%s/%s.jpg"%(self.img_dir,now)
        self.cam.capture(fname)
        self.last_image = fname
        return now

    def angle(self, val):
        return int(round(self.servo_min + (90.0 - val) * (self.servo_max - self.servo_min)/180.))

    def save(self):
        self.trajectory.append(self.current_pos)
        traj = [{"tilt": i.tilt, "pan": i.pan, "move": 60, "stay":10} 
                for i in self.trajectory]
        dico = {"trajectory": traj
                "delay": 60,
                "avg_awb":200,
                "avg_speed":6,
                "avg_speed_nb_img":3}

        with open(self.traj_file,"w") as f:
            f.write(json.dumps(dico, indent=4))
        return self.move()

def main():
    parser = ArgumentParser("serveur", description="open a web server on a given directory")
    parser.add_argument("-p", "--port", help="open specified port", type=int, default=8080)
    parser.add_argument("-i", "--ip", help="Listen on this port", type=str, default="0.0.0.0")
    parser.add_argument("-d", "--directory", help="Directory containing images", type=str, default="images")
    args = parser.parse_args()
    server = Server( img_dir=args.directory, ip=args.ip, port=args.port)
    server.start()

if __name__== '__main__' :
    main()
