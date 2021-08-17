#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

class Safety(object):
    #The class that handles emergency braking.

    def __init__(self):
        #Publisher for /brake
        self.pub1 = rospy.Publisher('brake', AckermannDriveStamped, queue_size=1)
        #Publisher for /brake_bool
        self.pub2 = rospy.Publisher('brake_bool', Bool, queue_size=1)
        #Subscribers to /odom and /scan
        self.sub1 = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.sub2 = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)

    def odom_callback(self, odom_msg):
        #Get latest speed/velocity
        self.vel = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        #Calculate number of laser datapoints
        n = len(scan_msg.ranges)
        #Create an array of all the beam angles
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, n) 

        #Calculating rdot
        rdot = [self.vel*np.cos(i) for i in angles]
        rdot = np.clip(rdot, 0, None)

        #Calculating ttc
        ttc = np.divide(scan_msg.ranges, rdot)

        #------------------------------------------------------
        # Checking ttc and braking if necessary
        if 0 < np.min(ttc) < 0.4:
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0

            self.pub1.publish(brake_msg)
            self.pub2.publish(True)



def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()


if __name__ == '__main__':
    main()
