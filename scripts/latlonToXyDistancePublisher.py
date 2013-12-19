#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('simple')
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

Latitude = 0.0
Longitude  = 0.0
endLatitude = 0.0
endLongitude = 0.0
compass_heading = 0.0

def get_distance(a_lat, a_lon, b_lat, b_lon):
#returns a distance (in meters) given two lat/lon coordinates

	earth_radius = 6371000

	x = earth_radius * (b_lon - a_lon) * math.pi / 180 * math.cos(a_lat * math.pi / 180)
	y = earth_radius * (b_lat - a_lat) * math.pi / 180

	#pythagorean theorem
	distance = math.sqrt(x*x+y*y)

	return distance

def get_heading(a_lat, a_lon, b_lat, b_lon):
#returns a compass heading (in degrees) given two lat/lon coordinates
#North: 0 degrees
#East: 90 degrees
#South: 180 degrees
#West: 270 degrees

	earth_radius = 6371000

	x = earth_radius * (b_lon - a_lon) * math.pi / 180 * math.cos(a_lat * math.pi / 180)
	y = earth_radius * (b_lat - a_lat) * math.pi / 180
	
	#avoid divide by 0 error by detecting perfect north or south heading	
	if x == 0:
		if y > 0:
			return 0 #North
		if y < 0:
			return 180 #South
		else:
			return -1 #lat/lon for a and b are the same; no heading could be determined

	#calculate the angle created by the xy coordinates
	angle = abs(math.degrees(math.atan(y/x)))

	#format the angle based on graph quadrant to be consistent with compass headings (0 degrees = North etc.)
	if x > 0:
		if y >= 0:
			return 90 - angle
		else:
			return 90 + angle
	else:
		if y >= 0:
			return 270 + angle
		else:
			return 270 - angle 


def xy_distance_publisher():
    global endLatitude, endLongitude
    pubDistance = rospy.Publisher('gpsdistance', Float64)
    pubHeading = rospy.Publisher('gpsheading', Float64)

    xydistance = get_distance(Latitude, Longitude, endLatitude, endLongitude)
    goal_heading = get_heading(Latitude, Longitude, endLatitude, endLongitude)       
#    print "Goal: ", goal_heading
#    print "Comp: ",compass_heading
#    print "Dist: ", xydistance
    turn_angle = goal_heading - compass_heading
    
    if turn_angle > 180:
        turn_angle = turn_angle - 360

    if turn_angle < -180:
        turn_angle = turn_angle + 360

    if turn_angle < 0:
        print "L: ",(turn_angle*-1)
    else:
       print "R: ",turn_angle

    # convert to radians before publishing
    turn_angle = turn_angle * (math.pi/180)
    pubDistance.publish(Float64(xydistance))
    pubHeading.publish(Float64(turn_angle))
    rospy.sleep(1.0)

def gpsdatacallback(data):
    global Latitude, Longitude
    Latitude = data.latitude
    Longitude = data.longitude
  #  print "\n Latitude - ", Latitude," Longitude - ", Longitude," \n"

def calculate_compass_heading(orientation):
    dir = 2*math.asin(orientation)

    if orientation<0:
        dir = dir+2*math.pi
    dir_degrees = 360 - dir * (180/math.pi)    
    return dir_degrees

def imuCallBack(data):
    global compass_heading
    compass_heading = calculate_compass_heading(data.orientation.z)

def gps_position_listener():
    rospy.Subscriber("/android/fix", NavSatFix, gpsdatacallback)

def imu_listener():
    rospy.Subscriber("/android/imu", Imu, imuCallBack)

if __name__ == '__main__':
    try:
	global endLatitude, endLongitude
	rospy.init_node('latlonToXyDistancePublisher', anonymous=True)
	gps_position_listener()
        imu_listener()
	f = open('gps_coordinates.txt')
	lines = f.readlines()
	f.close()
	string = str(lines)
	string = string[2:-4]
	stringList = string.split(",")
        stringList[1].strip(" ")
        endLatitude = float(stringList[0])
        endLongitude = float(stringList[1])
        #print "\n Desired Latitude - ", endLatitude, " Desired Longitude - ", endLongitude
	while not rospy.is_shutdown():
	    xy_distance_publisher()
    except rospy.ROSInterruptException:
        pass

