#!/usr/bin/python 
import sys
import os
import time
import picamera
import datetime  
import atexit
from signal import SIGTERM
from threading import Semaphore
import RPi.GPIO as GPIO
import daemon

class Daemon:
    """
    A generic daemon class.
    
    Usage: subclass the Daemon class and override the run() method
    """
    def __init__(self, pidfile, stdin='/dev/null', stdout='/dev/null', stderr='/dev/null'):
        self.stdin = stdin
        self.stdout = stdout
        self.stderr = stderr
        self.pidfile = pidfile
    
    def daemonize(self):
        """
        do the UNIX double-fork magic, see Stevens' "Advanced 
        Programming in the UNIX Environment" for details (ISBN 0201563177)
        http://www.erlenstar.demon.co.uk/unix/faq_2.html#SEC16
        """
        try: 
            pid = os.fork() 
            if pid > 0:
                # exit first parent
                sys.exit(0) 
        except OSError, e: 
            sys.stderr.write("fork #1 failed: %d (%s)\n" % (e.errno, e.strerror))
            sys.exit(1)
    
        # decouple from parent environment
        os.chdir("/") 
        os.setsid() 
        os.umask(0) 
    
        # do second fork
        try: 
            pid = os.fork() 
            if pid > 0:
                # exit from second parent
                sys.exit(0) 
        except OSError, e: 
            sys.stderr.write("fork #2 failed: %d (%s)\n" % (e.errno, e.strerror))
            sys.exit(1) 
    
        # redirect standard file descriptors
        sys.stdout.flush()
        sys.stderr.flush()
        si = file(self.stdin, 'r')
        so = file(self.stdout, 'a+')
        se = file(self.stderr, 'a+', 0)
        os.dup2(si.fileno(), sys.stdin.fileno())
        os.dup2(so.fileno(), sys.stdout.fileno())
        os.dup2(se.fileno(), sys.stderr.fileno())
    
        # write pidfile
        atexit.register(self.delpid)
        pid = str(os.getpid())
        file(self.pidfile,'w+').write("%s\n" % pid)
    
    def delpid(self):
        os.remove(self.pidfile)

    def start(self):
        """
        Start the daemon
        """
        # Check for a pidfile to see if the daemon already runs
        try:
            pf = file(self.pidfile,'r')
            pid = int(pf.read().strip())
            pf.close()
        except IOError:
            pid = None
    
        if pid:
            message = "pidfile %s already exist. Daemon already running?\n"
            sys.stderr.write(message % self.pidfile)
            sys.exit(1)
        
        # Start the daemon
        self.daemonize()
        self.run()

    def stop(self):
        """
        Stop the daemon
        """
        # Get the pid from the pidfile
        try:
            pf = file(self.pidfile,'r')
            pid = int(pf.read().strip())
            pf.close()
        except IOError:
            pid = None
    
        if not pid:
            message = "pidfile %s does not exist. Daemon not running?\n"
            sys.stderr.write(message % self.pidfile)
            return # not an error in a restart

        # Try killing the daemon process    
        try:
            while 1:
                os.kill(pid, SIGTERM)
                time.sleep(0.1)
        except OSError, err:
            err = str(err)
            if err.find("No such process") > 0:
                if os.path.exists(self.pidfile):
                    os.remove(self.pidfile)
            else:
                print(str(err))
                sys.exit(1)

    def restart(self):
        """
        Restart the daemon
        """
        self.stop()
        self.start()

    def run(self):
        """
        You should override this method when you subclass Daemon. It will be called after the process has been
        daemonized by start() or restart().
        """
        pass


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


class CameraDaemon(Daemon):
    def __init__(self, pidfile, stdin='/dev/null', stdout='/dev/null', stderr='/dev/null', img_folder="/tmp", led_id=0, pir_id=0):
        Daemon.__init__(self, pidfile, stdin, stdout, stderr)
        self.spy = None
        self.img_folder = img_folder
        self.led_id = led_id
        self.pir_id = pir_id
    
    def run(self):
        self.spy = Spy(self.img_folder, self.led_id, self.pir_id)
        self.spy.setup()
        while True:
            time.sleep(1)
            self.log("ping")

if __name__ == "__main__":
    folder = "/home/jerome/www"
    sensor = 23
    led = 18
    pid_file = '/var/log/pir.pid'
    #daemon = CameraDaemon('/var/log/pir.pid', img_folder=folder, led_id=led,pir_id=sensor)
    spy = Spy(folder, led, sensor)
    with daemon.DaemonContext():
        spy.setup()
        while True:
            time.sleep(10)
            spy.log("ping")

"""
if len(sys.argv) == 2:
        if 'start' == sys.argv[1]:
            daemon.start()
        elif 'stop' == sys.argv[1]:
            daemon.stop()
        elif 'restart' == sys.argv[1]:
            daemon.restart()    
        elif "test" == sys.argv[1]:
            s = Spy(folder, led, sensor)
            s.setup()
            raw_input("Enter to quit")
        else:
            print( "Unknown command")
            sys.exit(2)
#        sys.exit(0)
    else:
        print("usage: %s start|stop|restart|test" % sys.argv[0])
        sys.exit(2)
"""
