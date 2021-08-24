#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 12 # TODO
ki =  0.0# TODO
kd =  0.01# TODO
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 360  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.55  # meters
DESIRED_DISTANCE_LEFT = 0.90
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.10  # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub =  rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)# Subscribe to LIDAR
        self.drive_pub =  rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)# Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right

        #Hokuyo node starts scan from -45 --> +45 in simulator's axis
        scan_data = list(data.ranges)
        for i in scan_data:
            if np.isnan(i) == True:
                i = 0
            elif np.isnan(i) == False:
                pass

        if angle < -45 or angle > 225:
            raise ValueError('Angle not in range')
            return 0.0
        else:
            j = (len(scan_data)/ANGLE_RANGE)*(int(angle)+90)
            return scan_data[j]
            #error
        #n = len(scan_msg.ranges)
        #angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, n)
        #scan_data = data.ranges
        #scan_data = dict(zip(angles, scan_data))
        #scan_data = [scan_data.values(i) if scan_data.keys(i)<]

        #rem_indexes = 135*3
        #car_left = list(scan_range[1080:540+rem_indexes:-1])
        #car_right = list(scan_range[540-rem_indexes:0:-1])

        #new_range = car_right + car_left
        #print(new_range)


        #print(scan_range[135*3])

#        print(len(car_right), len(car_left))

        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for
        print('error = ', error)
        print('old err = ', prev_error)

        angle += np.deg2rad(kp*error + ki*integral + kd*(prev_error-error))

        print('angle =', angle)
        integral += error
        prev_error = error

        if -np.deg2rad(10) < angle < np.deg2rad(10):
            velocity = 1.5
        elif np.deg2rad(20) < angle < np.deg2rad(20):
            velocity = 1
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm
        theta = 60
        b = self.getRange(data, 180)
        a = self.getRange(data, 180-theta)
        
        theta = np.deg2rad(theta)
        alpha = np.arctan((a*np.cos(theta)-b) /
                          (a*np.sin(theta)))

        dt = b*np.cos(alpha)
        dt1 = dt + CAR_LENGTH*np.sin(alpha)

        et = leftDist-dt1

        return -et

    def lidar_callback(self, data):
        """ 
        """
          # TODO: replace with error returned by followLeft
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
