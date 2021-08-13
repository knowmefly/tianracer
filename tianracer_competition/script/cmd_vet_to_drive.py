#! /usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)

def callback(data):
    wheelbase = 1.0
    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering
    drive_msg.drive.speed=v
    drive_pub.publish(drive_msg)
if __name__ == '__main__': 
    try:
        rospy.init_node("cmd_vet_to_drive")
        scan_sub = rospy.Subscriber('/cmd_vel', Twist, callback)
        drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass    