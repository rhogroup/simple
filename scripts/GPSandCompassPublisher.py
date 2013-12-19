#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('simple')
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

endLatitude = 0.0
endLongitude = 0.0
compass_heading = 0.0
compass_points = []
gps_lat = 0.0
gps_lon = 0.0
gps_latitudes = []
gps_longitudes = []

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
	elif y < 0:
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

def calculate_compass_heading(orientation): #convert from IMU data to compass direction (in degrees)
    dir = 2*math.asin(orientation)

    if orientation<0: #if the angle is negative, add 2pi to make it positive
        dir = dir+2*math.pi
    dir_degrees = 360 - dir * (180/math.pi) #convert to degrees and format so that North is 0

    return dir_degrees

def distance_publisher():
    global endLatitude, endLongitude
    gps_distance = get_distance(gps_lat, gps_lon, endLatitude, endLongitude)
    print "Dist: ", gps_distance
    pubDistance = rospy.Publisher('gpsdistance', Float64)
    pubDistance.publish(Float64(gps_distance))

def compass_publisher():
    global endLatitude, endLongitude
    pubHeading = rospy.Publisher('gpsheading', Float64)
    
    goal_heading = get_heading(gps_lat, gps_lon, endLatitude, endLongitude)       
    print "Goal: ", goal_heading
    print "Comp: ",compass_heading
    turn_angle = goal_heading - compass_heading
  
    if turn_angle > 180:
        turn_angle = turn_angle - 360

    if turn_angle < -180:
        turn_angle = turn_angle + 360

#    if turn_angle < 0:
#        print "L: ",(turn_angle*-1)
#    else:
#           print "R: ",turn_angle

    # convert to radians before publishing
    turn_angle = turn_angle * (math.pi/180)
    pubHeading.publish(Float64(turn_angle))

def gps_CallBack(data):
    global gps_lat
    global gps_lon
    global gps_latitudes
    global gps_longitudes

    latitude = data.latitude
    longitude = data.longitude
    
    if len(gps_latitudes) == 3:
#        print "Lat",gps_latitudes
#	print "Lon",gps_longitudes
        gps_lat = sum(gps_latitudes) / 3.0
        gps_lon = sum(gps_longitudes) / 3.0
        del gps_latitudes[:]
        del gps_longitudes[:]
        distance_publisher()
    else:
        gps_latitudes.append(latitude)
        gps_longitudes.append(longitude)
    rospy.sleep(1)

def compass_CallBack(data):
    global compass_heading
    global compass_points

    if len(compass_points) == 3:
	print "Comp: ",compass_points
        compass_heading = sum(compass_points) / 3.0
        del compass_points[:]
	compass_publisher()
    else:
        compass_points.append(calculate_compass_heading(data.orientation.z))
    rospy.sleep(1)

def gps_position_listener():
    rospy.Subscriber("/android/fix", NavSatFix, gps_CallBack)

def imu_listener():
    rospy.Subscriber("/android/imu", Imu, compass_CallBack)

if __name__ == '__main__':
    try:
	global endLatitude, endLongitude
	rospy.init_node('GPSandCompassPublisher', anonymous=True)
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
	    pass
    except rospy.ROSInterruptException:
        pass

