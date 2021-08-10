#!/usr/bin/env python3

# math imports
import math
from math import inf, radians, sin, cos

import matplotlib.pyplot as plt

import numpy
from numpy import linspace, meshgrid

# import ROS
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Info to print graph
N = 128

ranx = [-500, 500]
rany = [-500, 500]

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

test_points = []

def curve(X, Y, points):
    cone = Cone(0, 0)
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

def ski(f, org, des):
    k = 1
    probe = org
    pl = [probe]
    i = 0
    while vec_dist(probe, des) > k * 2 and i < 300:
        a = f(probe[0], probe[1], des, test_points)
        probe = (probe[0] + math.cos(a) * k, probe[1] + math.sin(a) * k)
        pl += [probe]
        i += 1

    return pl

def test():
    # origin, destiny = (0, 0), (75, 65) # 0, 0 and X0, Y0
    curve_map = curve(X, Y, test_points)
    # curve_points = ski(curve_l, origin, destiny)

    # line_x = [curve_points[i][0]*130/300 for i in range(len(curve_points))]
    # line_y = [curve_points[i][1]*150/300 for i in range(len(curve_points))]
    
    # plt.plot(line_x, line_y, color="black")
    plt.imshow(curve_map, cmap='viridis', aspect=130/150, origin='lower')
    plt.colorbar()
    plt.tight_layout()
    plt.savefig("Heatmap.png")

def scan_callback(msg):
    global scan
    scan = msg.ranges

SPEED = 0.3

def timer_callback():
    # print(f'0*: {scan[0]}, 90*: {scan[90]}, 180*: {scan[180]}, 270*: {scan[270]}')
    # msg = Twist()

    # arc = scan[:50] + scan[310:]

    arc = [(i, n) for i , n in enumerate(scan) if n != inf]
    arc_points = [(-sin(radians(n[0])) * n[1], cos(radians(n[0])) * n[1]) for n in arc]

    k = 500
    global test_points
    test_points = [(n[0] * k, n[1] * k) for n in arc_points]
    test()
    exit()

    # distance_ahead = min(arc)

    # print(distance_ahead)

    # if distance_ahead < 0.2:
    #     msg.linear.x = -SPEED

    # elif distance_ahead < 0.4:
    #     msg.angular.z = SPEED

    # else:
    #     msg.linear.x = SPEED

    # publisher.publish(msg)

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

    timer = node.create_timer(0.5, timer_callback)
    timer

    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
