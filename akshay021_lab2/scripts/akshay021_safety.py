#!/usr/bin/env python
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

mag_rel_vel = dir_rel_vel = 0
ttc = 999999

def odom_callback(odom_msg):
    global mag_rel_vel, dir_rel_vel


    w = odom_msg.pose.pose.orientation.w
    mag_rel_vel = odom_msg.twist.twist.linear.x
    dir_rel_vel = 2*np.arccos(w)

def scan_callback(scan_msg):
    global ttc

    # TODO: calculate TTC
    n = int((scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment) + 1

    rdot = [(mag_rel_vel*np.cos(0-(i*scan_msg.angle_increment))) for i in range(-n/2,n/2)]
    rdot = np.clip(rdot, 0, None)

    ttc = np.divide(scan_msg.ranges, rdot)
    pubs()

    # TODO: publish brake message and publish controller bool
    
def pubs(): 
    pub1 = rospy.Publisher('brake', AckermannDriveStamped, queue_size=1)
    pub2 = rospy.Publisher('brake_bool', Bool, queue_size=1)
    
    print(0 < np.min(ttc) < 5)
    if 0 < np.min(ttc) < 5:
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = 0
        
        pub1.publish(ack_msg)
        pub2.publish(True)
    else:
        pass
        
def main():
    rospy.init_node('safety_node')
    sub1 = rospy.Subscriber('odom', Odometry, odom_callback)
    sub2 = rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.spin()
    


if __name__ == '__main__':
    main()
