#!/usr/bin/env python

import rospy
import math
import numpy as np
from dino_nav.msg import Stat
import matplotlib.pyplot as plt
import pickle

seq = 0

def callback(data):
    global seq

    points = []
    polar = []
    angle = data.scan.angle_max + math.pi*3/2
    for d in data.scan.ranges:
        x = math.cos(angle)*d*10
        y = math.sin(angle)*d*10
        points.append((x,y))
        polar.append((d, angle))
        angle -= data.scan.angle_increment


    dim = 100
    vl = dim

    grid = np.zeros([dim, dim])
    for p in points:
        x = dim/2 + p[0]
        y = (dim - dim/6) + p[1]
        if(x >= 0 and x <dim and y >=0 and y<dim):
            grid[int(y)][int(x)] = 1

    pts = np.array(points, np.float32)
    pol = np.array(polar, np.float32)
    grd = np.array(grid, np.float32)
    act = np.array( [data.steer, data.throttle] , np.float32)
    pickle.dump({"lidar": pol, "bitmap": grd, "actuators": act, "onroad": True }, open("dataset/data" + str(seq), "w"))
    seq += 1

if __name__ == '__main__':
    rospy.init_node('dataset')
    rospy.Subscriber("dinonav/stat", Stat, callback)
    rospy.spin()

