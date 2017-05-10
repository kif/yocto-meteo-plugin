#!/usr/rbin/python
import numpy
from scipy.signal import convolve, gaussian

g19_2 = gaussian(19, 2)
g19_2 /= g19_2.sum()
i=numpy.arange(40)
j=0.5**(0.25*i)
k = ((j+0.099)/1.099)**(1/0.45)*(235-16)+16
m2=j<0.018
k[m2] = (235-16)/4.5*j[m2]+16
kr = numpy.round(k).astype(int)
ukr = numpy.concatenate(([0],numpy.sort(numpy.unique(kr)),[256]))
start = -0.25*(ukr.size-1)+0.5
delta_expo = numpy.arange(start,0.5, 0.25)

yuv = numpy.fromfile(open("image.yuv"),"uint8")
def calc_expo(yuv):
    yprime = yuv[:2592*1944]
    cnt = numpy.bincount(yprime)
    cnt = numpy.bincount(yprime, minlength=256)
    cs = numpy.zeros(257, int)
    cs[1:] = numpy.cumsum(cnt)
    dev = cs[ukr[1:]]-cs[ukr[:-1]]
    cnv = convolve(dev, g19_2, mode="same")
    idx = cnv.argmax()
    return delta_expo[idx]-0.5*(cnv[idx+1]-cnv[idx-1])/(cnv[idx+1]+cnv[idx-1]-2*cnv[idx])
print(calc_expo(yuv))

class Frame(object):
    """Object representing an image with all associated metadata"""
    def __init__(self, data, resolution):
        self.timestamp = time.time()
        self.data = np.frombuffer(data, dtype=numpy.uint8).copy()
        self.width, self.height = resolution
        self.fwidth = (self.width + 31) & ~31
        self.fheight = (self.height + 15) & ~15
        self.sem = Semaphore()
        self._y = None
        self._yuv = None
        self._rgb = None
        self.over_exposed = None
        self.under_exposed = None
        self.gain_r = None
        self.gain_g = None
        self.gain_b = None
        self.gain_d = None
        self.speed = None
        self.gx = None
        self.gy = None
        self.gz = None





