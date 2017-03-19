#!/usr/bin/env python
#coding: utf-8


from __future__ import division, print_function
from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image
import Adafruit_PCA9685
import os
import glob
import bottle
import threading
from argparse import ArgumentParser
import inotify.adapters
from collections import namedtuple

Position = namedtuple("Position", ("pan", "tilt"))


bottle.debug(True)
root = os.path.dirname(os.path.abspath(__file__))

class Server(object):
    def __init__(self, img_dir="images", ip="0.0.0.0", port=80, timer=30):
        self.img_dir = img_dir
        self.ip = ip
        self.port = port
        self.timer = timer
        self.all_images = []
        self.common_files = os.path.join( os.path.dirname(os.path.abspath(__file__)), "bottle")
        self.bottle = bottle.Bottle()
        self.setup_routes()
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.servo_id_tilt = 15
        self.servo_id_pan = 14
        self.default_pos = Position(0, 90)
        self.current_pos = self.default_pos

    def update_all_images(self):
        self.all_images = [ i for i in os.listdir(self.img_dir) if i.endswith(".jpg")]
        self.all_images.sort()
        threading.Thread(target=self.start_notify).start()

    def start_notify(self):
        i = inotify.adapters.Inotify()
        i.add_watch(self.img_dir)
        try:
            for event in i.event_gen():
                if event is not None:
                    header, type_names, watch_path, filename = event
                    if watch_path == self.img_dir and "IN_CLOSE_WRITE" in type_names and filename.endswith(".jpg"):
                        if filename != self.all_images[-1]:
                            self.all_images.append(filename)
        finally:
            i.remove_watch(self.img_dir)

    def setup_routes(self):
        self.bottle.route('/css/:filename', callback=self.render_css)
        self.bottle.route('/robots.txt', callback=self.robots)
        self.bottle.route('/images/:filename', callback=self.server_static)
        self.bottle.route('/previous', callback=self.previous)
        self.bottle.route('/next', callback=self.next)
        self.bottle.route('/first', callback=self.first)
        self.bottle.route('/last', callback=self.last)
        self.bottle.route('/', callback=self.index)

    def render_css(self, filename):
        return bottle.static_file(filename, root=self.common_files)

    def robots(self):
        return bottle.static_file("robots.txt", root=self.common_files)

    def server_static(self, filename):
        return bottle.static_file(filename, root=os.path.join(root, self.img_dir))
    
    def previous(self):
        idx = (int(bottle.request.cookies.get('idx', '-1')) - 1) % len(self.all_images)
        bottle.response.set_cookie('idx', str(idx))
        return self.show(idx)

    def next(self):
        idx = (int(bottle.request.cookies.get('idx', '-1')) + 1) % len(self.all_images)
        bottle.response.set_cookie('idx', str(idx))
        return self.show(idx)

    def first(self):
        idx = 0
        bottle.response.set_cookie('idx', str(idx))
        return self.show(idx)

    def last(self):
        idx = len(self.all_images) - 1
        bottle.response.set_cookie('idx', str(idx))
        return self.show(idx)

    def index(self):
        idx = int(bottle.request.cookies.get('idx', '-1')) % len(self.all_images)
        return self.show(idx)

    def show(self, idx):
        template_file = os.path.join(self.common_files, 'interface.html')
        img = self.all_images[idx]
        date_time = os.path.splitext(self.all_images[idx])[0].split("-")
        date_time = "/".join(date_time[-2::-1])+" "+date_time[-1]
        return bottle.template(template_file,
                               date_time=date_time,
                               image="images/"+img)

    def start(self):
        """Start the serveur (does not return):::"""
        self.bottle.run(host=self.ip, port=self.port)

    def move(self)
        self.goto_pos(self.current_pos)

    def goto_pos(self, pos):
                pan, tilt = pos
        self.pwm.set_pwm(self.servo_id_pan, 0, self.angle(pan))
        self.pwm.set_pwm(self.servo_id_tilt, 0, self.angle(tilt))

    def pan_min(self):
        self.curent_pos = Position(-90, self.current_pos.tilt)
        self.move()

    def pan_max(self):
        self.curent_pos = Position(+90, self.current_pos.tilt)
        self.move()
    
    def tilt_min(self):
        self.curent_pos = Position(self.current_pos.pan, -50)
        self.move()

    def tilt_max(self):
        self.curent_pos = Position(self.current_pos.pan, +90)
        self.move()



def main():
    parser = ArgumentParser("serveur", description="open a web server on a given directory")
    parser.add_argument("-t", "--timer", help="delay before refreshing directory list", type=int, default=30)
    parser.add_argument("-p", "--port", help="open specified port", type=int, default=8080)
    parser.add_argument("-i", "--ip", help="Listen on this port", type=str, default="0.0.0.0")
    parser.add_argument("-d", "--directory", help="Directory containing images", type=str, default="images")
    args = parser.parse_args()
    server = Server( img_dir=args.directory, ip=args.ip, port=args.port, timer=args.timer)
    server.update_all_images()
    server.start()

if __name__== '__main__' :
    main()
