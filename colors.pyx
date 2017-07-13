#cython: language_level=3, boundscheck=False, wraparound=False, initializedcheck=False, cdivision=True

import cython
import numpy
cimport numpy 
from libc.math cimport sqrt, ceil, floor


cpdef int pseudo_dist(int x, int y) nogil:
    """Calculate sqrt(x*x + y*y) in an approximate way, using only integers
    http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
    """
    cdef int mini, maxi, approx
    if x < 0:
        x = -x
    if y < 0:
        y = -y
    if x < y:
        mini = x
        maxi = y
    else:
        mini = y
        maxi = x
    approx = maxi * 1007 + mini * 441
    if maxi < (mini << 4):
        approx -= maxi * 40
    return (approx + 512) >> 10
    

def yuv420_to_yuv(stream, resolution):
    """Convert a YUV420 linear stream into an image YUV444
    
    :param stream: string (bytes) with the stream
    :param resolution: 2-tuple (width, height)
    :return: YUV array + historgram of Y
    """
    cdef:
        int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, y
        numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
        numpy.uint8_t[:, :, ::1] yuv
        int[::1] histo
    
    histo = numpy.zeros(256, dtype=numpy.int32)
    
    width, height = resolution
    fwidth = (width + 31) & ~(31)
    fheight = (height + 15) & ~ (15)
    ylen = fwidth * fheight
    uvlen = ylen // 4
    assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
    yuv = numpy.empty((height, width, 3), dtype=numpy.uint8)
    with nogil:
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
    
    
def yuv420_to_rgb8(stream, resolution):
    """Convert a YUV420 linear stream into an image RGB
    array: 
    [[1.164383  0  1.596027
     [1.164383 -0.391762 -0.812968
     [1.164383 2.017232 0 
    
    :param stream: string (bytes) with the stream
    :param resolution: 2-tuple (width, height)
    :return: YUV array + historgram of Y,R,G,B
    """
    cdef:
        int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, y, u, v, r, g, b
        float uf, yf, vf
        numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
        numpy.uint8_t[:, :, ::1] rgb
        int[:, ::1] histo
    
    histo = numpy.zeros((4, 256), dtype=numpy.int32)
    
    width, height = resolution
    fwidth = (width + 31) & ~(31)
    fheight = (height + 15) & ~ (15)
    ylen = fwidth * fheight
    uvlen = ylen // 4
    assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
    rgb = numpy.empty((height, width, 3), dtype=numpy.uint8)
    with nogil:
        for i in range(height):
            k = fwidth * i
            l = (fwidth // 2) * (i // 2)
            for j in range(width):
                m = j // 2
                y = cstream[k + j]
                u = cstream[ylen + l + m]
                v = cstream[ylen + uvlen + l + m]
                histo[0, y] += 1
                # integer version (*65535)
                #y -= 16
                #y = 0 if y < 0 else (219 if y > 219 else y)                
                #u -= 128
                #v -= 128
                #y *= 262144
                #r = (y + 104597 * v) >> 16
                #g = (y - 25675 * v - 53278 * u) >> 16
                #b = (y + 132201 * u) >> 16
                
                # Floating point version
                yf = 1.164383 * (<float> y - 16.)
                uf = <float> u - 128.0
                vf = <float> v - 128.0
                r = <numpy.uint8_t> (yf + 1.596027 * vf)
                g = <numpy.uint8_t> (yf - 0.812968 * vf - 0.391762 * uf)
                b = <numpy.uint8_t> (yf + 2.017232 * uf)
                
                # clip to the 0-255 range
                r = 0 if r < 0 else (255 if r > 255 else r)
                g = 0 if g < 0 else (255 if g > 255 else g)
                b = 0 if b < 0 else (255 if b > 255 else b)
               
                rgb[i, j, 0] = r
                rgb[i, j, 1] = g
                rgb[i, j, 2] = b
                histo[1, r] += 1
                histo[2, g] += 1
                histo[3, b] += 1
            
    return numpy.asarray(rgb), numpy.asarray(histo)


def yuv420_to_rgb16(stream, resolution):
    """Convert a YUV420 linear stream into a gamma compressed image RGB16
    array: 
    [[1.164383  0  1.596027
     [1.164383 -0.391762 -0.812968
     [1.164383 2.017232 0 
    
    :param stream: string (bytes) with the stream
    :param resolution: 2-tuple (width, height)
    :return: YUV array + historgram of Y,R,G,B
    """
    cdef:
        int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, 
        int y, u, v, r, g, b, half_width, half_height
        float rd, cr, cg, cb, position, fp, cp, delta_hi, delta_low, rf, gf, bf
        float yf, uf, vf, rg, gg, bg, gamma
        int ys, rv, gu, gv, bu 
        numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
        numpy.uint16_t[:, :, ::1] rgb
        int[:, ::1] histo
    
    histo = numpy.zeros((4, 256), dtype=numpy.int32)
    
    # Coef for Y'UV -> R'G'B'
    ys = <int> ((1 << 16) - 1) / (235 - 16) #1.164
    rv = <int> (((1 << 16) - 1) * 1.596027 / ((1 << 8) - 1) + 0.5)
    gu = <int> (((1 << 16) - 1) * 0.391762 / ((1 << 8) - 1) + 0.5)
    gv = <int> (((1 << 16) - 1) * 0.812968 / ((1 << 8) - 1) + 0.5)
    bu = <int> (((1 << 16) - 1) * 2.017232 / ((1 << 8) - 1) + 0.5)
    
    width, height = resolution
    half_width = width // 2
    half_height = height // 2
    fwidth = (width + 31) & ~(31)
    fheight = (height + 15) & ~ (15)
    ylen = fwidth * fheight
    uvlen = ylen // 4
    assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
    rgb = numpy.empty((height, width, 3), dtype=numpy.uint16)
    with nogil:
        for i in range(height):
            k = fwidth * i
            l = (fwidth // 2) * (i // 2)
            for j in range(width):
                m = j // 2
                y = cstream[k + j]
                u = cstream[ylen + l + m]
                v = cstream[ylen + uvlen + l + m]
                histo[0, y] += 1
                y -= 16
                if y < 0: #Saturated black
                    rgb[i, j, 0] = 0
                    rgb[i, j, 1] = 0
                    rgb[i, j, 2] = 0
                    histo[1, 0] += 1
                    histo[2, 0] += 1
                    histo[3, 0] += 1
                    continue
                elif y > 219: #Saturated white
                    rgb[i, j, 0] = 65535
                    rgb[i, j, 1] = 65535
                    rgb[i, j, 2] = 65535
                    histo[1, 255] += 1
                    histo[2, 255] += 1
                    histo[3, 255] += 1
                    continue
    
                u -= 128
                v -= 128
                
                # integer version (*65535) 
                y *= ys
                #add half of the offset to cope for rounding error
                r = (y + rv * v)   
                g = (y - gv * v - gv * u)
                b = (y + bu * u)
                
                #Clip to 0-65535
                r = 0 if r < 0 else (65535 if r > 65535 else r)
                g = 0 if g < 0 else (65535 if g > 65535 else g)
                b = 0 if b < 0 else (65535 if b > 65535 else b)

                histo[1, (r + (1 << 7)) >> 8] += 1
                histo[2, (g + (1 << 7)) >> 8] += 1
                histo[3, (b + (1 << 7)) >> 8] += 1
                
                
                #Normalization
                r = 0 if r < 0 else (65535 if r > 65535 else r)
                g = 0 if g < 0 else (65535 if g > 65535 else g)
                b = 0 if b < 0 else (65535 if b > 65535 else b)

                rgb[i, j, 0] = r
                rgb[i, j, 1] = g
                rgb[i, j, 2] = b
                
    return numpy.asarray(rgb), numpy.asarray(histo)


cdef class SRGB:
    """Compress an image in sRGB space
    https://en.wikipedia.org/wiki/SRGB
    
    gamma: 2.4 offset: 0.055, slope:12.92
    """
    cdef:
        readonly numpy.uint8_t[::1] LUT
    def __cinit__(self):
        cdef:
            float c, a=0.055, res
            int i, s 
        self.LUT = numpy.empty(1<<16, dtype=numpy.uint8)
        with nogil:
            for i in range(1<<16):
                c = <float>i / 65535.
                if c < 0.0031308:
                    res = <int>(12.92 * c *255 + 0.5)
                else:
                    res = (((1+a)*c**(1/2.4) - a) * 255 + 0.5)
                self.LUT[i] = <int> res

    def __dealloc__(self):
        self.LUT = None
    
    def compress(self, numpy.uint16_t[:, :, ::1] inp):
        """Compress a RGB16 linear into a sRGB8 image"""
        cdef: 
            int width, height, i, j, k
            numpy.uint8_t[:, :, ::1] out
        
        height = inp.shape[0]
        width = inp.shape[1]
        out = numpy.empty((height, width, 3), dtype=numpy.uint8)
        with nogil:
            for i in range(height):
                for j in range(width):
                    for k in range(3):
                        out[i, j, k] = self.LUT[inp[i, j, k]]
        return numpy.asarray(out)

    @staticmethod
    def sum(numpy.uint16_t[:, :, ::1] im1, numpy.uint16_t[:, :, ::1] im2):
        "sum two images and flag if overflow"
        cdef: 
            bint overflow = False
            int width, height, i, j, k, r
            numpy.uint16_t[:, :, ::1] out
        
        height = im1.shape[0]
        width = im1.shape[1]
        out = numpy.empty((height, width, 3), dtype=numpy.uint16)
        with nogil:
            for i in range(height):
                for j in range(width):
                    for k in range(3):
                        r = im1[i, j, k] + im2[i, j, k]
                        if r > 65535: 
                            overflow = True
                            out[i, j, k] = 65535
                        else:
                            out[i, j, k] = r
        return numpy.asarray(out), overflow
        
cdef class Flatfield:
    cdef: 
        readonly  numpy.uint16_t[::1] lut_r, lut_g, lut_b, LUT 
        int nbits
        
    def __cinit__(self, flatfile, nbits=14):
        self.nbits = nbits
        self.LUT = self.calc_gamma()
        self.lut_r, self.lut_g, self.lut_b = self.calc_colors(flatfile, nbits=self.nbits)      

    def __dealloc__(self):
        #self.radius = None
        #self.red = None
        #self.green = None
        #self.blue = None
        #self.data = None
        self.LUT = None
        self.lut_r = None
        self.lut_g = None
        self.lut_b = None

    def calc_gamma(self, float a=0.099, float slope=4.5, float gamma=1.0 / 0.45, float clim=0.081):
        #rg/4.5 if rg<=0.081 else ((rg+0.099)/1.099)**(gamma)
        cdef:
            float c, res 
            int i
        print("initialize the gamma LUT") 
        LUT = numpy.empty(1 << 16, dtype=numpy.uint16)
        for i in range(1 << 16):
            c = i / 65535.0
            if c < clim:
                res = (i / slope) + 0.5
            else:
                res = 65535.0 * ((c + a) / (1.0 + a)) ** (gamma) + 0.5
            LUT[i] = <numpy.uint16_t> res
        return LUT

    def calc_colors(self, flatfile, int nbits=14):
        """Initalizes the tree colors LUTs, on 14 bits"""
        cdef:
            double[:, :] data
            int i, j, dmax, d, scale, size
            numpy.uint16_t[::1] lut_r, lut_g, lut_b, count
            float cr, cb, cg, rd, position, cp, fp, delta_low, delta_hi, rmin, rmax, dr
            float[::1] radius, sred, sblue, sgreen, red, green, blue
        print("initialize the color LUT")
        data = numpy.loadtxt(flatfile)
        radius = numpy.ascontiguousarray(data[:, 0], dtype=numpy.float32)
        sred = numpy.ascontiguousarray(data[:, 1], dtype=numpy.float32)
        sgreen = numpy.ascontiguousarray(data[:, 2], dtype=numpy.float32)
        sblue = numpy.ascontiguousarray(data[:, 3], dtype=numpy.float32)

        size = data.shape[0]
        rmin = radius[0]
        rmax = radius[size - 1]
        dr = (rmax - rmin) / (size - 1)
        
        dmax = int(ceil(rmax))
        count = numpy.zeros(dmax + 1, dtype=numpy.uint16)
        lut_r = numpy.zeros(dmax + 1, dtype=numpy.uint16)
        lut_g = numpy.zeros(dmax + 1, dtype=numpy.uint16)
        lut_b = numpy.zeros(dmax + 1, dtype=numpy.uint16)
        red = numpy.zeros(dmax + 1, dtype=numpy.float32)
        green = numpy.zeros(dmax + 1, dtype=numpy.float32)
        blue = numpy.zeros(dmax + 1, dtype=numpy.float32)
        for i in range(dmax):
            for j in range(dmax):
                d = pseudo_dist(i, j)
                if d > dmax:
                    continue
                d = max(d, 0)

                rd = sqrt(<float>(i * i + j * j))
                if rd <= rmin:
                    cr = sred[0]
                    cg = sgreen[0] 
                    cb = sblue[0] 
                elif rd >= rmax:
                    cr = sred[size - 1]
                    cg = sgreen[size - 1] 
                    cb = sblue[size - 1] 
                else:
                    position = (rd - rmin) / dr
                    cp = ceil(position)
                    fp = floor(position)
                    if cp == fp:
                        cr = sred[<int> cp]
                        cg = sgreen[<int> cp] 
                        cb = sblue[<int> cp] 
                    else: #Bilinear interpolation
                        delta_low = position - fp
                        delta_hi = cp - position
                        cr = sred[<int> fp] * delta_hi + sred[<int> cp] * delta_low
                        cg = sgreen[<int> fp] * delta_hi + sgreen[<int> cp] * delta_low
                        cb = sblue[<int> fp] * delta_hi + sblue[<int> cp] * delta_low
                red[d] += cr
                blue[d] += cb
                green[d] += cg
                count[d] += 1
        scale = (1 << nbits) - 1
        for d in range(dmax + 1):
            if count[d] == 0:
                continue
            cr = red[d] * scale / count[d] + 0.5
            cg = green[d] * scale / count[d] + 0.5
            cb = blue[d] * scale / count[d] + 0.5
            
            lut_r[d] = <int> cr
            lut_g[d] = <int> cg
            lut_b[d] = <int> cb
        return lut_r, lut_g, lut_b
                
    def yuv420_to_rgb16(self, stream, resolution):
        """Convert a YUV420 linear stream into an image RGB
        array: 
        [[1.164383  0  1.596027
         [1.164383 -0.391762 -0.812968
         [1.164383 2.017232 0 
        
        :param stream: string (bytes) with the stream
        :param resolution: 2-tuple (width, height)
        :return: YUV array + historgram of Y,R,G,B
        """
        cdef:
            int i, j, k, l, m, width, height, fwidth, fheight, ylen, uvlen, d
            int y, u, v, r, g, b, half_width, half_height
            float rd, cr, cg, cb, position, fp, cp, delta_hi, delta_low, rf, gf, bf
            float yf, uf, vf, rg, gg, bg, gamma
            int ys, rv, gu, gv, bu
            numpy.uint8_t[::1] cstream = numpy.fromstring(stream, numpy.uint8)
            numpy.uint16_t[:,:,::1] rgb
            int[:, ::1] histo
        
        histo = numpy.zeros((4, 256), dtype=numpy.int32)
        
        #Coef for Y'UV -> R'G'B'
        #ys = 65535 / (235-16) #1.164
        #rv = 65535 * 1.596 / 255
        #gu = -0.392 * 65535 / 255
        #gv = -0.813 * 65535 / 255
        #bu = 2.017 * 65535 / 255
        #gamma = 1/0.45
        # Coef for Y'UV -> R'G'B'
        ys = <int> ((1 << 16) - 1) / (235 - 16) #1.164
        rv = <int> (((1 << 16) - 1) * 1.596027 / ((1 << 8) - 1) + 0.5)
        gu = <int> (((1 << 16) - 1) * 0.391762 / ((1 << 8) - 1) + 0.5)
        gv = <int> (((1 << 16) - 1) * 0.812968 / ((1 << 8) - 1) + 0.5)
        bu = <int> (((1 << 16) - 1) * 2.017232 / ((1 << 8) - 1) + 0.5)
        
        width, height = resolution
        half_width = width //2
        half_height = height//2
        fwidth = (width + 31) & ~(31)
        fheight = (height + 15) & ~ (15)
        ylen = fwidth * fheight
        uvlen = ylen // 4
        assert cstream.size >= (ylen + 2 * uvlen), "stream is len enough"
        rgb = numpy.empty((height, width, 3), dtype=numpy.uint16)
        with nogil:
            for i in range(height):
                k = fwidth * i
                l = (fwidth // 2) * (i // 2)
                for j in range(width):
                    m = j // 2
                    y = cstream[k + j]
                    u = cstream[ylen + l + m]
                    v =cstream[ylen + uvlen + l + m]
                    histo[0, y] += 1
                    y -= 16
#                    y=0 if y<0 else (219 if y>219 else y)
                    if y < 0: #Saturated black
                        rgb[i, j, 0] = 0
                        rgb[i, j, 1] = 0
                        rgb[i, j, 2] = 0
                        histo[1, 0] += 1
                        histo[2, 0] += 1
                        histo[3, 0] += 1
                        continue
                    elif y > 219: #Saturated white
                        rgb[i, j, 0] = 65535
                        rgb[i, j, 1] = 65535
                        rgb[i, j, 2] = 65535
                        histo[1, 255] += 1
                        histo[2, 255] += 1
                        histo[3, 255] += 1
                        continue

                    u -= 128
                    v -= 128
                    
                    # integer version (*65535) 
                    y *= ys
                    #add half of the offset to cope for rounding error
                    r = (y + rv * v)   
                    g = (y - gv * v - gv * u)
                    b = (y + bu * u)

                    # integer version (*65535) 
                    #y *= 262144
                    #add half of the offset to cope for rounding error
                    #r = (y + 104597 * v + (1<<7) ) >> 8 
                    #g = (y - 53278 * v - 25675 * u + (1<<7)) >> 8
                    #b = (y + 132201 * u + (1<<7)) >> 8
                    
                    #Clip to 0-65535
                    r = 0 if r<0 else (65535 if r>65535 else r)
                    g = 0 if g<0 else (65535 if g>65535 else g)
                    b = 0 if b<0 else (65535 if b>65535 else b)

                    histo[1, (r + (1<<7)) >> 8] += 1
                    histo[2, (g + (1<<7)) >> 8] += 1
                    histo[3, (b + (1<<7)) >> 8] += 1
                    
                    #Conversion to linear scale: faster done with LUT
                    #rf = rg/4.5 if rg<=0.081 else ((rg+0.099)/1.099)**(gamma)
                    #gf = gg/4.5 if gg<=0.081 else ((gg+0.099)/1.099)**(gamma)
                    #bf = bg/4.5 if bg<=0.081 else ((bg+0.099)/1.099)**(gamma)
                    
#                     #Conversion to linear scale using a log scale
#                     r = self.LUT[r]
#                     g = self.LUT[g]
#                     b = self.LUT[b]
#                     
#                     #Flatfield correction
#                     rd = sqrt(<float>((i-half_height)**2 + (j-half_width)**2))
#                     if rd <= self.rmin:
#                         cr = self.red[0]
#                         cg = self.green[0] 
#                         cb = self.blue[0] 
#                     elif rd >= self.rmax:
#                         cr = self.red[self.size - 1]
#                         cg = self.green[self.size - 1] 
#                         cb = self.blue[self.size - 1] 
#                     else:
#                         position = (rd - self.rmin) / self.dr
#                         cp = ceil(position)
#                         fp = floor(position)
#                         if cp == fp:
#                             cr = self.red[<int>cp]
#                             cg = self.green[<int>cp] 
#                             cb = self.blue[<int>cp] 
#                         else: #Bilinear interpolation
#                             delta_low = position - fp
#                             delta_hi = cp - position
#                             cr = self.red[<int>fp]*delta_hi + self.red[<int>cp]*delta_low
#                             cg = self.green[<int>fp]*delta_hi + self.green[<int>cp]*delta_low
#                             cb = self.blue[<int>fp]*delta_hi + self.blue[<int>cp]*delta_low
#                     r = <int>(r * cr + 0.5)
#                     g = <int>(g * cg + 0.5)
#                     b = <int>(b * cb + 0.5)
                    
                    #Flatfield correction using LUT table
                    d = pseudo_dist((i - half_height), (j - half_width))
                    r = (self.LUT[r] * self.lut_r[d] + (1<<(self.nbits-1))) >> self.nbits
                    g = (self.LUT[g] * self.lut_g[d] + (1<<(self.nbits-1))) >> self.nbits
                    b = (self.LUT[b] * self.lut_b[d] + (1<<(self.nbits-1))) >> self.nbits
                    
                    #Conversion to gamma scale
                    #rg = rf*4.5 if rf<=0.018 else (rf**0.45)*1.099 - 0.099
                    #gg = gf*4.5 if gf<=0.018 else (gf**0.45)*1.099 - 0.099
                    #bg = bf*4.5 if bf<=0.018 else (bf**0.45)*1.099 - 0.099
                    
                    #Normalization
                    r = 0 if r<0 else (65535 if r>65535 else r)
                    g = 0 if g<0 else (65535 if g>65535 else g)
                    b = 0 if b<0 else (65535 if b>65535 else b)

                    rgb[i, j, 0] = r
                    rgb[i, j, 1] = g
                    rgb[i, j, 2] = b
                    
        return numpy.asarray(rgb), numpy.asarray(histo)

    def yuv420_to_yuv(self, stream, resolution):
        return yuv420_to_yuv(stream, resolution)
