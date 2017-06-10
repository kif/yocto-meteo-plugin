#cython: language_level=3, boundscheck=False, wraparound=False, initializedcheck=False

import cython
import numpy
cimport numpy 

def yuv420_to_yuv(stream, resolution):
    """Convert a YUV420 linear stream into an image YUV444
    
    :param stream: string (bytes) with the stream
    :param resolution: 2-tuple (width, height)
    :return: YUV array + historgram of Y
    """
    cdef:
        int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, y
        numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
        numpy.uint8_t[:,:,::1] yuv
        int[::1] histo
    
    histo = numpy.zeros(256, dtype=numpy.int32)
    
    width, height = resolution
    fwidth = (width + 31) & ~(31)
    fheight = (height + 15) & ~ (15)
    ylen = fwidth * fheight
    uvlen = ylen // 4
    assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
    yuv = numpy.empty((height, width, 3), dtype=numpy.uint8)
    for i in range(height):
        k = fwidth * i
        l = (fwidth // 2) * (i // 2)
        for j in range(width):
            m = j // 2
            y = cstream[k + j]
            yuv[i, j, 0] = y
            yuv[i, j, 1] = cstream[ylen + l + m]
            yuv[i, j, 2] = cstream[ylen + uvlen + l + m]
            histo[y] += 1
    return numpy.asarray(yuv), numpy.asarray(histo)
    
def yuv420_to_rgb(stream, resolution):
    """Convert a YUV420 linear stream into an image RGB
    
    :param stream: string (bytes) with the stream
    :param resolution: 2-tuple (width, height)
    :return: YUV array + historgram of Y,R,G,B
    """
    cdef:
        int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, y, u, v, r, g, b
        #float yf, uf, vf
        numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
        numpy.uint8_t[:,:,::1] rgb
        int[:, ::1] histo
    
    histo = numpy.zeros((4,256), dtype=numpy.int32)
    
    width, height = resolution
    fwidth = (width + 31) & ~(31)
    fheight = (height + 15) & ~ (15)
    ylen = fwidth * fheight
    uvlen = ylen // 4
    assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
    rgb = numpy.empty((height, width, 3), dtype=numpy.uint8)
    for i in range(height):
        k = fwidth * i
        l = (fwidth // 2) * (i // 2)
        for j in range(width):
            m = j // 2
            y = cstream[k + j]
            u = cstream[ylen + l + m]
            v =cstream[ylen + uvlen + l + m]
            histo[0, y] += 1
            # integer version (*4096)
            y -= 16
            y *= 4768
            u -= 128
            v -= 128
            r = (y + 6537 * v) >> 12
            g = (y - 3330 * v - 1606 * u) >> 12
            b = (y + 8262 * u) >> 12
            
            # Floating point version
            #yf = 1.164 * <float> y
            #uf = <float> u
            #vf = <float> v
            #r = <numpy.uint8_t> (yf + 1.596 * vf)
            #g = <numpy.uint8_t> (yf - 0.813 * vf - 0.392 * uf)
            #b = <numpy.uint8_t> (yf + 2.017 * uf)
            
            if r < 0:
                r = 0
            elif r > 255:
                r = 255
            if b < 0:
                b = 0
            elif b > 255:
                b = 255
            if g < 0:
                g = 0
            elif g > 255:
                g = 255
            
            rgb[i, j, 0] = r
            rgb[i, j, 1] = g
            rgb[i, j, 2] = b
            histo[1, r] += 1
            histo[2, g] += 1
            histo[3, b] += 1
            
    return numpy.asarray(rgb), numpy.asarray(histo)
