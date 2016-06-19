#!/usr/bin/env python
#coding: utf-8


import os
import glob
import bottle
import threading
from argparse import ArgumentParser

parser = ArgumentParser("serveur", description="open a web server on a given directory")
parser.add_argument("-t", "--timer", help="delay before refreshing directory list", type=int, default=30)
args = parser.parse_args()

TIMER = args.timer
sem = threading.Semaphore()
bottle.debug(True)
root = os.path.dirname(os.path.abspath(__file__))
img_dir = "images"
all_images = glob.glob("%s/*/*.jpg"%img_dir)
all_images.sort()

def update_all_images():
    #print("update -->%s images"%len(all_images))
    _, last_day, last_img = all_images[-1].split(os.sep)
    days = os.listdir(img_dir)
    days.sort()
    try:
        start = days.index(last_day)
    except exception as err:
        print("error in finding last_day: %s"%err)
    else:
        with sem:
            for day in days[start:]:
                times = os.listdir(os.path.join(img_dir,day))
                times.sort()
                for one in times:
                    if not one.endswith(".jpg"):
                        continue
                    full = os.path.join(img_dir,day, one)
                    if not full in all_images:
                        all_images.append(full)
    start_timer()

def start_timer():
    threading.Timer(TIMER, update_all_images).start()

@bottle.route('/css/:filename')
def render_css(filename):
    return bottle.static_file(filename, root=root)

@bottle.route('/robots.txt')
def robots():
    return bottle.static_file("robots.txt", root=root)

@bottle.route('/images/<filepath:path>')
def server_static(filepath):
    #print(filepath)
    return bottle.static_file(filepath, root=os.path.join(root,"images"))

@bottle.route('/previous')
def previous():
    idx = (int(bottle.request.cookies.get('idx', '-1')) - 1) % len(all_images)
    bottle.response.set_cookie('idx', str(idx))
    return show(idx)

@bottle.route('/next')
def next():
    idx = (int(bottle.request.cookies.get('idx', '-1')) + 1) % len(all_images)
    bottle.response.set_cookie('idx', str(idx))
    return show(idx)

@bottle.route('/')
def index():
    idx = int(bottle.request.cookies.get('idx', '-1')) % len(all_images)
    return show(idx)

def show(idx):
    #print(idx)
    template_file = 'interface.html'
    date_time = " ".join(os.path.splitext(all_images[idx])[0].split("/")[1:3])
    return bottle.template(template_file,
                           date_time=date_time,
                           image=all_images[idx])

def main():
    start_timer()
    bottle.run(host="0.0.0.0", port=8080)

if __name__== '__main__' :
    main()
