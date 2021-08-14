#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from akshay021_lab1.msg import scan_range

min_val = max_val = 0

#The callback fuunction for scan data
def compute(msg):
    global min_val, max_val

    laser_data = msg.ranges
    max_val = np.max(laser_data)
    min_val = np.min(laser_data)
    
#Subscriber that takes data from /scan
def laser_datain():
    sub = rospy.Subscriber('scan', LaserScan, compute)
    
#Publishers that publish to all 3 required topics
def minmaxout():
    pubmin = rospy.Publisher('closest_point', Float64, queue_size=1)
    pubmax = rospy.Publisher('farthest_point', Float64, queue_size=1)
    pubboth = rospy.Publisher('scan_range', scan_range, queue_size=1)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pubmin.publish(min_val)
        pubmax.publish(max_val)
        pubboth.publish(maximum_value = max_val, minimum_value = min_val)

        rate.sleep()

#Main
if __name__ == '__main__':
    rospy.init_node('lidar_processing')

    try:
        laser_datain()
        minmaxout()
    except rospy.ROSInterruptException:
        pass
