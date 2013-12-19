// This file subscribes to /fix message from the Arduino playground package
// This file reads the latitude and longitude values continuously and stores in a file
// The format it stores it in is as follows:
// "latitude , longitude" (in separate lines)
// Keval Patel


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include <fstream>
#include <limits>


#define PI 						(3.1415926536)
#define RADIUS_OF_EARTH 		(6371000)


#define STORE_GPS 				(keyboardInput == 's')
#define DONE_STORING_GPS		(keyboardInput == 'x')

using namespace std;
long double Latitude, Longitude;


//Store latitude and longitude when a GPS message is received
void gpsLocationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	static int sequence = 1;
	long double latitude_hardcoded;
	Latitude = (*msg).latitude;
	Longitude = (*msg).longitude;

	cout << std::setprecision(12);
	std::cout << std::endl << "Sequence:- " << sequence << std::endl;
	std::cout << "Latitude = " << Latitude << std::endl;
	std::cout << "Longitude = " << Longitude << std::endl;
	sequence++;
}

void calculateXDistance()
{
	// X = RADIUS_OF_EARTH * (longitude_end - longitude_origin) * (PI / 180) * cos (latitude_origin * (PI/180))
	// Y = RADIUS_OF_EARTH * (latitude_end - latitude_origin) * (PI / 180)

}

int main(int argc, char **argv)
{

	 /**
	   * The ros::init() function needs to see argc and argv so that it can perform
	   * any ROS arguments and name remapping that were provided at the command line. For programmatic
	   * remappings you can use a different version of init() which takes remappings
	   * directly, but for most command-line programs, passing argc and argv is the easiest
	   * way to do it.  The third argument to init() is the name of the node.
	   *
	   * You must call one of the versions of ros::init() before using any other
	   * part of the ROS system.
	   */
	  ros::init(argc, argv, "gps_points_collect_all");

	  /**
	    * NodeHandle is the main access point to communications with the ROS system.
	    * The first NodeHandle constructed will fully initialize this node, and the last
	    * NodeHandle destructed will close down the node.
	    */
	  ros::NodeHandle n;

	  // /fix is the topic on which the GPS unit publishes longitude and latitude
	  // We are creating a subscriber which can hear to messages
	  ros::Subscriber subscriber_for_fix = n.subscribe("/android/fix", 1, gpsLocationCallback);


	  /**
	    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	    * callbacks will be called from within this thread (the main one).  ros::spin()
	    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	    */
	  ros::spinOnce();
	  ofstream gpsFile;
	  char keyboardInput;
	  gpsFile.open ("androidphoneallGpsPointsTest_11_07_2013.txt");
	  while (ros::ok())
	  {
		  sleep(1);
		  ros::spinOnce();
		  gpsFile << std::setprecision(12) << Latitude << ", " << Longitude << endl;
		  //if (DONE_STORING_GPS)
		  //{
		  //	  gpsFile.close();
		  //	  break;
		  // }
	  }
	  gpsFile.close();
	  std::cout << std::endl << "done" << std::endl;
}
