#!/usr/bin/env python3

# math imports
import math
from math import degrees, inf, pi, radians, sin, cos

import matplotlib.pyplot as plt

import numpy
from numpy import linspace, meshgrid

# import ROS
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Info to print graph
N = 100

ranx = [-200, 200]
rany = [-200, 200]

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

    A = 200
    C = -1
    sigmaX = 20
    sigmaY = 20
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

    k = 2500

    def __init__(self, X0, Y0):
        self.X0 = X0
        self.Y0 = Y0

    def fun(self, X, Y):
        return numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

    def fun_x(self, X, Y):
        return self.k * (X - self.X0) / numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

    def fun_y(self, X, Y):
        return self.k * (Y - self.Y0) / numpy.sqrt(self.k * ((X - self.X0)**2 + (Y - self.Y0)**2))

test_points = []

def curve(X, Y, points, des):
    cone = Cone(des[0], des[1])
    z = cone.fun(X, Y)
    for p in points:
        gau = Gauss(p[0], p[1])
        z += gau.fun(X, Y)
    return z

def curve_l(X, Y, destiny, points):
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

def full_ski(f, org, des, test_points):
    k = 1
    probe = org
    pl = [probe]
    i = 0
    while vec_dist(probe, des) > k * 2 and i < 1000:
        a = f(probe[0], probe[1], des, test_points)
        probe = (probe[0] + math.cos(a) * k, probe[1] + math.sin(a) * k)
        pl += [probe]
        i += 1

    return pl

def test(test_points):
    destiny = (0, 200) # 0, 0 and X0, Y0
    curve_map = curve(X, Y, [(t[0]*1.5, t[1]*1.5) for t in test_points], destiny)

    ang = curve_l(0, 0, destiny, test_points)
    print(degrees(ang))

    plt.imshow(curve_map, cmap='viridis', aspect=1, origin='lower')
    plt.colorbar()
    plt.tight_layout()
    plt.savefig("Heatmap.png")

def scan_callback(msg):
    global scan
    scan = msg.ranges

SPEED = 0.1

def timer_callback():

    arc = [(i, n) for i , n in enumerate(scan) if n != inf]
    arc_points = [(-sin(radians(n[0])) * n[1], cos(radians(n[0])) * n[1]) for n in arc]

    k = 100
    points = [(n[0] * k, n[1] * k) for n in arc_points]
    # test(points)

    ang = curve_l(0, 0, (0, 200), points)

    msg = Twist()

    msg.linear.x = SPEED
    msg.angular.z = (ang-pi/2)/2.5

    publisher.publish(msg)

def main(args=None):

    global scan
    scan = []

    rclpy.init(args=args)

    global node
    node = rclpy.create_node('wanderbot')

    global publisher
    publisher = node.create_publisher(Twist, 'cmd_vel', rclpy.qos.qos_profile_system_default)
    sub = node.create_subscription(LaserScan, 'scan', scan_callback, rclpy.qos.qos_profile_sensor_data)
    sub

    timer = node.create_timer(0.05, timer_callback)
    timer

    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
