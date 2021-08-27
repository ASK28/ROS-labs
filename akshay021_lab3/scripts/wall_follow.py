#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 11 # TODO
ki = 0.0 # TODO
kd = 0.5  # TODO
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 360  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.55  # meters
DESIRED_DISTANCE_LEFT = 0.8
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.prev_time = float(str(rospy.Time.now()))

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)  # TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)  # TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement

        scan_data = list(data.ranges)

        for i in scan_data:
            if np.isnan(i) == True:
                i = 0
            elif np.isnan(i) == False:
                pass

        if angle < -90 or angle > 270:
            raise ValueError('Angle not in range')
            return 0.0
        else:
            j = (len(scan_data)/ANGLE_RANGE)*(int(angle)+90)
            return scan_data[j]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for

        curr_time = float(str(rospy.Time.now()))        

        angle = np.deg2rad(kp*error + ki*integral + kd*((error-prev_error)/(curr_time-self.prev_time)))

        if angle < np.deg2rad(-24):
            angle = np.deg2rad(-24)
        elif angle > np.deg2rad(24):
            angle = np.deg2rad(24)
        else:
            pass

        print('error = ', error)
        print('old err = ', prev_error)
        print(rospy.Time.now(), ' angle =', angle)
        integral += error
        prev_error = error
        self.prev_time = curr_time

        if np.deg2rad(-10) < angle < np.deg2rad(10):
            velocity = 1.5
        elif np.deg2rad(-20) < angle < np.deg2rad(20):
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
        #TODO:implement
        theta = 60
        b = self.getRange(data, 180)
        a = self.getRange(data, 180-theta)
        print(a,b)

        theta = np.deg2rad(theta)
        alpha = np.arctan((a*np.cos(theta)-b)/(a*np.sin(theta)))

        dt = b*np.cos(alpha)
        dt1 = dt + CAR_LENGTH*np.sin(alpha)

        et = leftDist-dt1

        return -et

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)  # TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)
        #print('left =', data.ranges[800:820])
        #print('right =', data.ranges[260:280])
        #print('fl =', data.ranges[620:640])
        #print('fr =', data.ranges[500:520])
        #print(data.ranges[810])


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    #rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
