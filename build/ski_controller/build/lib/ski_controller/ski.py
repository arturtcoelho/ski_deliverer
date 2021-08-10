#!/usr/bin/python3

# math imports
import math

import matplotlib.pyplot as plt

import numpy
from numpy import linspace, meshgrid

# Info to print graph
N = 64

ranx = [0, 150]
rany = [0, 130]

x = linspace(ranx[0], ranx[1], N)
y = linspace(rany[0], rany[1], N)

X, Y = meshgrid(x, y)

X0 = 0
Y0 = 0

class Gauss():

    """
    This class contains a gaussian curve, it's function
    and it's x and y derivatives

    A = Amplitude
    C = Constant
    sigma = 'wideness' of curve (standart deviation)
    """

    A = 150
    C = -1
    sigmaX = 16
    sigmaY = 16
    sigmaX2 = sigmaX**2
    sigmaY2 = sigmaY**2

    def __init__(self, X0, Y0):
        self.X0 = X0
        self.Y0 = Y0

    def fun(self, X, Y):
        x_part = (X - self.X0)**2 / self.sigmaX2
        y_part = (Y - self.Y0)**2 / self.sigmaY2
        Z = self.A * numpy.exp(self.C * (x_part + y_part))
        return Z

    def fun_x(self, X, Y):
        mult = 2*(X - self.X0) / self.sigmaX2
        x_part = (X - self.X0)**2 / self.sigmaX2
        y_part = (Y - self.Y0)**2 / self.sigmaY2
        return self.A * mult * numpy.exp(self.C * (x_part + y_part)) 

    def fun_y(self, X, Y):
        mult = 2*(Y - self.Y0) / self.sigmaY2
        x_part = (X - self.X0)**2 / self.sigmaX2
        y_part = (Y - self.Y0)**2 / self.sigmaY2
        return self.A * mult * numpy.exp(self.C * (x_part + y_part)) 

class Cone():

    """
    This class contains a cone function and its derivatives
    """

    k = 100

    def __init__(self, X0, Y0):
        self.X0 = X0
        self.Y0 = Y0

    def fun(self, X, Y):
        return numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

    def fun_x(self, X, Y):
        return self.k * (X - self.X0) / numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

    def fun_y(self, X, Y):
        return self.k * (Y - self.Y0) / numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

# added gaussian curves
# points = [(50, i*20) for i in range(5)]
points = [(20, 20), (40, 30)]

def curve(X, Y):
    cone = Cone(75, 65)
    z = cone.fun(X, Y)
    for p in points:
        gau = Gauss(p[0], p[1])
        z += gau.fun(X, Y)
    return z

def curve_l(X, Y, destiny):
    cone = Cone(destiny[0], destiny[1])
    zx = -cone.fun_x(X, Y)
    zy = -cone.fun_y(X, Y)
    for p in points:
        gau = Gauss(p[0], p[1])
        zx += gau.fun_x(X, Y)
        zy += gau.fun_y(X, Y)

    return math.atan2(zy, zx)

def vec_dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2) 

def ski(f, org, des):
    k = 1
    probe = org
    pl = [probe]
    i = 0
    while vec_dist(probe, des) > k * 2 and i < 300:
        a = f(probe[0], probe[1], des)
        probe = (probe[0] + math.cos(a) * k, probe[1] + math.sin(a) * k)
        pl += [probe]
        i += 1

    return pl


def run_test():

    origin, destiny = (0, 0), (75, 65) # 0, 0 and X0, Y0
    curve_map = curve(X, Y)
    curve_points = ski(curve_l, origin, destiny)

    line_x = [curve_points[i][0]*130/300 for i in range(len(curve_points))]
    line_y = [curve_points[i][1]*150/300 for i in range(len(curve_points))]
    
    plt.plot(line_x, line_y, color="black")
    plt.imshow(curve_map, cmap='viridis', aspect=130/150, origin='lower')
    plt.colorbar()
    plt.tight_layout()
    plt.savefig("Heatmap.png")

if __name__ == '__main__':
    run_test()
