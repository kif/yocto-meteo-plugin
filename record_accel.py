#!/usr/bin/env python
import sys
import os
import threading
import time
import mma8451
import numpy
import h5py
from Queue import Queue
from math import sqrt
from collections import namedtuple

point = namedtuple("point", ["t", "g", "x", "y", "z"])
accel = mma8451.MMA8451()
accel.set_resolution("high")
queue = Queue()


class Recorder(threading.Thread):
    def __init__(self, dst_file, queue):
        threading.Thread.__init__(self, name="Recorder")
        self.dst_file = dst_file
        self.queue = queue
        self.chunk_size = 3600
        self.datset_name = "data"
        self.buffer = numpy.zeros((self.chunk_size, 5), dtype=numpy.float64)
        self.idx = 0
    
    def save_hdf5(self, dataset):
        print(dataset)
        assert dataset.shape[0] == self.chunk_size
        assert dataset.shape[1] == 5
        if not os.path.exists(self.dst_file):
            with h5py.File(self.dst_file) as h:
                ds = h.require_dataset(self.datset_name, shape=(self.chunk_size,5), dtype=numpy.float64,
                        chunks=(self.chunk_size,5), maxshape=(None,5),
                        compression="gzip", shuffle=True)
                ds.attrs["t"] = 0
                ds.attrs["g"] = 1
                ds.attrs["x"] = 2
                ds.attrs["y"] = 3
                ds.attrs["z"] = 4
                ds[:,:] = dataset
        else:
            with h5py.File(self.dst_file) as h:
                ds = h[self.datset_name]
                old_size = ds.shape[0]
                new_size = old_size + self.chunk_size
                ds.resize((new_size,ds.shape[1]))
                ds[old_size:new_size, :] = dataset
    
    def run(self):
        while True:
            p = self.queue.get()
            if p is None:
                #poison pill
                break
            self.buffer[self.idx] = p
            self.idx+=1
            print(self.idx, p)
            if self.idx == self.chunk_size:
                self.save_hdf5(self.buffer)
                self.idx = 0
                self.buffer[:, :] = 0
            self.queue.task_done()

recorder = Recorder(sys.argv[1], queue)
recorder.start()


def record():
    axes = accel.get_axes_measurement()
    t = time.time()
    g =  sqrt(axes['x']**2 + axes['y']**2 + axes['z']**2)
    p = point(t, g, axes['x'], axes['y'], axes['z'])
    queue.put(p)

while True:
    record()
