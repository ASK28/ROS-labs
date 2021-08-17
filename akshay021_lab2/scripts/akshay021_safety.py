#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

vel = 0
ttc = 999999

def odom_callback(odom_msg):
    global vel

    vel = odom_msg.twist.twist.linear.x

def scan_callback(scan_msg):
    global ttc
    # TODO: calculate TTC
    n = int((scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment) + 1

    rdot = [(vel*np.cos(-(i*scan_msg.angle_increment))) for i in range(-n/2,n/2)]
    rdot = np.clip(rdot, 0, None)

    ttc = np.divide(scan_msg.ranges, rdot)

    # TODO: publish brake message and publish controller bool
    
def pubs():
    pub1 = rospy.Publisher('brake', AckermannDriveStamped, queue_size=1)
    pub2 = rospy.Publisher('brake_bool', Bool, queue_size=1)
    rate = rospy.Rate(700)
    
    

    while not rospy.is_shutdown():
        if 0 < np.min(ttc) < 0.4:
            print(np.min(ttc))
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0
            
            pub1.publish(ack_msg)
            pub2.publish(True)
        else:
            rate.sleep()
        
def main():
    rospy.init_node('safety_node')

    try:
        sub1 = rospy.Subscriber('odom', Odometry, odom_callback, queue_size=1)
        sub2 = rospy.Subscriber('scan', LaserScan, scan_callback, queue_size=1)
        pubs()
    except rospy.ROSInterruptException:
        pass
    


if __name__ == '__main__':
    main()
