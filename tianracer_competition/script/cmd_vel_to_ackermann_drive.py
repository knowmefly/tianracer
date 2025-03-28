#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  # global ackermann_cmd_topic
  # global drive_pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  drive_msg = AckermannDriveStamped()
  drive_msg.drive.steering_angle=steering
  drive_msg.drive.speed=v
  drive_pub.publish(drive_msg)


if __name__ == '__main__': 
  try:
    # rospy.init_node('cmd_vel_to_ackermann_drive')
    rospy.init_node('test_node')
    wheelbase = 1.0
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    
    # # twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    # # ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    # # wheelbase = rospy.get_param('~wheelbase', 1.0)

    # twist_cmd_topic = '/cmd_vel'
    # ackermann_cmd_topic = '/ackermann_cmd'
    # wheelbase = 1.0
    # rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    # # pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
    # # drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    
    # rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, wheelbase)
    
    # rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

