#!/usr/bin/python 
import os
import RPi.GPIO as GPIO
import time
import picamera
import datetime  


class Spy(object):
    def __init__(self, folder, led_id, pir_id):
        self.folder = folder
        self.led = led_id
        self.pir = pir_id
        self.time_up = 0
        self.cam = None
        self.max_pict = 3
        self.delay = 1.0

    def callback(self, what):
        if GPIO.input(self.pir)==GPIO.LOW:
            print("Has been up %.3s"%(time.time()-self.time_up))
            return
        GPIO.output(self.led, GPIO.HIGH)
        self.time_up = time.time()
        now = datetime.datetime.now().strftime("%Y-%m-%d/%Hh%Mm%Ss")
        txt = "%s: %s"%(now, what)
        s_date,s_time = now.split("/")
        print(txt)
        with open("/var/log/pir.log","a") as f:
            f.write(txt+os.linesep)
        dirname = os.path.join(folder,s_date)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        for i in range(self.max_pict):
            if GPIO.input(self.pir):
                 fname = datetime.datetime.now().strftime("%Hh%Mm%Ss.jpg")
                 fname = os.path.join(dirname,fname)
                 self.cam.capture(fname)
                 time.sleep(self.delay)
            else:
                break
        GPIO.output(self.led, GPIO.LOW)

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pir, GPIO.IN, GPIO.PUD_DOWN)
        GPIO.setup(self.led, GPIO.OUT)
        GPIO.output(self.led, GPIO.LOW)
        self.cam = picamera.PiCamera()
        GPIO.add_event_detect(sensor, GPIO.BOTH, callback=self.callback, bouncetime=1000)

    def __del__(self):
        GPIO.cleanup()


if __name__ == "__main__":
    folder = "/home/jerome/www"
    sensor = 23
    led = 18
    s = Spy(folder, led, sensor)
    s.setup()
    raw_input("Enter to quit")

