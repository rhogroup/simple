#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('simple')
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def get_heading(orientation):
    dir = 2*math.asin(orientation)
    
    if orientation<0:
        dir = dir+2*math.pi
    
    return 360 - dir*180/math.pi

def imuCallBack(data):
    print "Raw Data: ", data.orientation.z
    print "Heading: ", get_heading(data.orientation.z)

def imu_listener():
    rospy.init_node('compassTest', anonymous=True)
    rospy.Subscriber("/android/imu", Imu, imuCallBack)

if __name__ == '__main__':
    try:
	imu_listener()
	while not rospy.is_shutdown():
	    pass
    except rospy.ROSInterruptException:
        pass

