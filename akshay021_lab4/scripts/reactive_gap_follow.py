#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools
import operator

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)  # TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)  # TODO

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        window_size = 3
        scan_deg = 120
        start_idx = (180-(scan_deg/2))*3
        
        proc_ranges = ranges[start_idx:start_idx+(3*scan_deg)+1]
        #proc_ranges = ranges[360:721]
        #print(len(proc_ranges))
        #proc_ranges = np.convolve(proc_ranges, np.ones(window_size)/window_size, mode='valid')

        #print(len(proc_ranges)) 
        
        #for i in range(0,len(ranges)):
        #    proc_ranges.append(float(np.average(ranges[i-1:i+2])))
        #print(proc_ranges)
#
        #proc_ranges = [3.0 if x > 3 else x for x in proc_ranges]
#
        closest = np.nanmin(proc_ranges)
        indices = [i for i,x in enumerate(proc_ranges) if x == closest]
        print(indices)

        radius = 1

        for i in indices:
            try:
                bubble = proc_ranges[i-radius:i+radius+1]
                proc_ranges[i-radius:i+radius+1] = np.zeros(len(bubble))
            except IndexError:
                pass

        #y = np.ones(len(proc_ranges)) (2*radius)+1

        #mask = np.convolve(proc_ranges==closest, [True]*(radius*2-1), mode='same')
        #print(mask)
        #y[mask[:-radius+1]] = 0.0


        #print(y)
        #proc_ranges *= y

        #bub_start = proc_ranges[:idx+radius+1]


        #[-(radius+2):] = np.zeros(radius)

        #proc_ranges[idx:radius+idx+1]
            #if 0 <= i =< len(proc_ranges): 
            #    proc_ranges[i] = 0.0
            #else:
            #    pass

        #indexes = []

        #for i in range(0, len(proc_ranges)):
        #    if proc_ranges[i] <= closest:
        #        for a in range(i-1,i+2):
        #            proc_ranges[a] = 0.0
                    #indexes.append(a)
            
        #for i in indexes:
        #    try:
        #        proc_ranges[i] = 0.0
        #    except IndexError:
        #        pass
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        '''
        gaps = [[]]
        j = 0
        for i in range(0,len(free_space_ranges)):
            if free_space_ranges[i] == 0:
                j += 1
            else:
                gaps[j].append(i)

        gap_lens = [len(i) for i in gaps]
        gaps[gap_lens.index(max(gap_lens))]
        print(gaps)
        '''
        #gaps = [[i for i,value in it] for key,it in itertools.groupby(enumerate(free_space_ranges), key=operator.itemgetter(1)) if key !=0.0]
        isnonzero = np.concatenate(([0], (np.asarray(free_space_ranges) != 0).view(np.int8), [0]))
        absdiff = np.abs(np.diff(isnonzero))
        gaps = list(np.where(absdiff == 1)[0].reshape(-1,2))

        #print(np.where(free_space_ranges != 0.0))

        #gaps = [[i for i,value in it] for is_true, it in itertools.groupby(enumerate(free_space_ranges), lambda x: x >= 0.0) if x != 0.0]
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
        #ranges = [1,1,3,5,19,3,3,0.2,6,7,7,np.inf,np.inf,5,5,4,4,4,0.8,0.2,0.5,2,3,0.2]
        best_point_ttl = drive_angle_ttl = 0

        #Find closest point to LiDAR
        #proc_ranges = self.preprocess_lidar(ranges)
#
        #max_gap_index = self.find_max_gap(proc_ranges)
        #best_point, drive_angle = self.find_best_point(
        #    max_gap_index[0], max_gap_index[1], proc_ranges)


        # Loop used to manually control rate of publishing
        for i in range(4):
            #Eliminate all points inside 'bubble' (set them to zero)
            proc_ranges = self.preprocess_lidar(ranges)

            #Find max length gap
            max_gap_index = self.find_max_gap(proc_ranges)
            #Find the best point in the gap
            best_point, drive_angle = self.find_best_point(max_gap_index[0], max_gap_index[1], proc_ranges)

            best_point_ttl += best_point
            drive_angle_ttl += drive_angle
        
        best_point = best_point_ttl/4
        drive_angle = drive_angle_ttl/4       
      
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = drive_angle
        drive_msg.drive.speed = 1.5
        self.drive_pub.publish(drive_msg)



def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
