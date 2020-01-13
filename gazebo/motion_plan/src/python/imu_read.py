#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped

def clbk_imu(msg):
    rospy.loginfo(msg)


def main():
    rospy.init_node('imu_read')
    
    sub = rospy.Subscriber('/imu',Imu,clbk_imu)
    
    rospy.spin()

if __name__ == '__main__':
    main()