#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('asb_fwd')

pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rate = rospy.Rate(10)
lin = Twist()

lin.angular.x = 20

while not rospy.is_shutdown():
    pub.publish(lin)
   



