#!/usr/bin/python 
import sys
import os
import time
import picamera
import datetime  
from threading import Semaphore
import RPi.GPIO as GPIO
import daemon
import signal


class Spy(object):
    def __init__(self, folder, led_id, pir_id):
        self.folder = folder
        self.led = led_id
        self.pir = pir_id
        self.time_up = 0
        self.cam = None
        self.max_pict = 3
        self.delay = 1.0
        self.logfile = "/var/log/pir.log"
        self.sem = Semaphore()

    def light(self, turn=None):
        """
        set to True to switch on the light
        """
        if turn:
             GPIO.output(self.led, GPIO.HIGH)
        else:
             GPIO.output(self.led, GPIO.LOW)

    def capture(self):
        """
        Take a picture with the camera
        """
        s_date = datetime.datetime.now().strftime("%Y-%m-%d")
        dirname = os.path.join(self.folder,s_date)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
        fname = datetime.datetime.now().strftime("%Hh%Mm%Ss.jpg")
        fname = os.path.join(dirname, fname)
        self.cam.capture(fname)

    def shoot(self):
        """
        Take a picture with light on
        """
        self.light(True)
        self.capture()
        self.light(False)

    def log(self, what):
        """ write a message in the logs"""
        if not what.endswith(os.linesep):
            what+=os.linesep
        with self.sem:
            with open(self.logfile,"a") as f:
                now = datetime.datetime.now().strftime("%Y-%m-%d/%Hh%Mm%Ss")
                f.write(now+": "+what)

    def callback(self, what):
        """
        Function called for every change on the PIR sensor pin
        """
        if GPIO.input(self.pir)==GPIO.LOW:
            print("Has been up %.3s"%(time.time()-self.time_up))
            return
        self.light(True)
        self.time_up = time.time()
        self.log(str(what))
        for i in range(self.max_pict):
            if GPIO.input(self.pir):
                 self.capture()
            else:
                break
        self.light(False)

    def callback_up(self, what):
        """
        Function called for every raise on the PIR sensor pin
        """
        self.light(True)
        self.log(str(what))
        for i in range(self.max_pict):
            if GPIO.input(self.pir):
                 self.capture()
            else:
                break
        self.light(False)

    def setup(self):
        """
        Initialize the GPIO and connect call-backs (starts a thread)
        """
        self.log("setup led=%s pir=%s"%(self.led, self.pir))
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pir, GPIO.IN, GPIO.PUD_DOWN)
        GPIO.setup(self.led, GPIO.OUT)
        GPIO.output(self.led, GPIO.LOW)
        self.cam = picamera.PiCamera()
        GPIO.add_event_detect(self.pir, GPIO.RISING, callback=self.callback_up, bouncetime=1000)
        #GPIO.add_event_detect(self.pir, GPIO.BOTH, callback=self.callback, bouncetime=1000)

    def __del__(self):
        GPIO.cleanup()


if __name__ == "__main__":
    folder = "/home/jerome/www"
    sensor = 23
    led = 18
    pid_file = '/var/run/pir.pid'
    
    def test(delay):
        spy = Spy(folder, led, sensor)
        spy.setup()
        while True:
           time.sleep(delay)
           spy.log("test")

    def start():
        if os.path.exists(pid_file):
            print("pid file %s exists, process already running under pid: %s"%(pid_file,open(pid_file).read()))
            sys.exit()

        spy = Spy(folder, led, sensor)
        with daemon.DaemonContext():
            with open(pid_file,"w") as f:
                f.write(str(os.getpid()))
            spy.setup()
            while True:
                time.sleep(10)
                spy.log("ping")
    
    def stop():
        if os.path.exists(pid_file):
            pid = int(open(pid_file).read())
            os.kill(pid,signal.SIGTERM)
            os.unlink(pid_file)
        else:
            print("Process not running")

    if len(sys.argv) == 2:
        if 'start' == sys.argv[1]:
            start()
        elif 'stop' == sys.argv[1]:
            stop()
        elif 'restart' == sys.argv[1]:
            stop()
            start()    
        elif "test" == sys.argv[1]:
            test(10)
        else:
            print( "Unknown command")
            sys.exit(2)
        sys.exit(0)
    else:
        print("usage: %s start|stop|restart|test" % sys.argv[0])
        sys.exit(2)
