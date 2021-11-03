#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.prev_angle = 0

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        window_size = 3 #Convolution window size
        scan_deg = 120 #Scan angle in front of car
        bubble_radius = 1 #Radius of safety bubble
        
        start_idx = (180-(scan_deg/2))*3

        proc_ranges = ranges[start_idx:start_idx+(3*scan_deg)+1]
        proc_ranges = np.convolve(proc_ranges, np.ones(window_size)/window_size, mode='valid')

        #proc_ranges = [3.0 if x > 3 else x for x in proc_ranges]

        closest = np.nanmin(proc_ranges)
        indices = [i for i,x in enumerate(proc_ranges) if x == closest]

        for i in indices:
            try:
                bubble = proc_ranges[i-bubble_radius:i+bubble_radius+1]
                proc_ranges[i-bubble_radius:i+bubble_radius+1] = np.zeros(len(bubble))
            except IndexError:
                pass
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        nonzero = np.concatenate(([0], (np.asarray(free_space_ranges) != 0).view(np.int8), [0]))
        abs_diff = np.abs(np.diff(nonzero))
        gaps = list(np.where(abs_diff == 1)[0].reshape(-1,2))

        gap_size = [len(i) for i in gaps]

        return gaps[gap_size.index(np.max(gap_size))]

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        point = list(ranges).index(np.max(ranges[start_i:end_i]))
        angle = np.deg2rad((((len(ranges)/2))/3)-point)

        return point, angle

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)

        #Find closest point to LiDAR
        #Eliminate all points inside 'bubble' (set them to zero)
        proc_ranges = self.preprocess_lidar(ranges)
        #Find max length gap
        max_gap_index = self.find_max_gap(proc_ranges)
        #Find the best point in the gap
        best_point, drive_angle = self.find_best_point(max_gap_index[0], max_gap_index[1], proc_ranges)
        
        #Average point and angle
        if np.abs(self.prev_angle-drive_angle) < np.deg2rad(15):
            drive_angle = np.average([drive_angle,self.prev_angle])
        else:
            pass
        
        
        self.prev_angle = drive_angle

        

        if np.deg2rad(-10) < drive_angle < np.deg2rad(10):
            velocity = 2
        elif np.deg2rad(-20) < drive_angle < np.deg2rad(20):
            velocity = 1.5
        else:
            velocity = 1
      
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = drive_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)



def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
