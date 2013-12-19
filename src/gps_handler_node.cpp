// This node subscribes to the GPS data from Arduino Playground package on topic /fix
// For now it is just printing the Longitude and Latitude data that it gets
// Following steps are to be followed for making this Node fully working:-
// 1. First make sure I am getting the data from the GPS module and print it out
// 2. Then add code to store the values to a file so that I can plot it on a map
//    This means that the correct GPS data is getting recorded by the GPS and we are storing it well
// 3. Then determine how I can get the precise comparison of data so that Goal reached can be signalled
// 4. Maybe the compass data can also be subscribed by this to be stored and published with Longitude and Latitude data
//
//
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


#define STORE_GPS 				(keyboardInput == 'a')
#define DONE_STORING_GPS		(keyboardInput == 'd')

using namespace std;
long double Latitude, Longitude;


//typedef std::numeric_limits< double > dbl;

double d = 3.14159265358979;

//Store latitude and longitude when a GPS message is recieved
void gpsLocationCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	static int sequence = 1;
	long double latitude_hardcoded;
	Latitude = (*msg).latitude;
	Longitude = (*msg).longitude;

	// cout.precision(dbl::digits10);
	// cout << "Pi: " << fixed << d << endl;
	// latitude_hardcoded = 42.3931183333;
	// cout << "\n Latitude Hardcoded - "<< fixed << latitude_hardcoded << endl;

	// Another way to do it
	// cout << std::setprecision (15) << "\n Latitude Hardcoded - "<< latitude_hardcoded << endl;
	// std::cout << std::setprecision (15) << 3.14159265358979 << std::endl;

	cout << std::setprecision(15);
	cout << "\nLatitude original - " << (*msg).latitude << endl;
	std::cout << std::endl << "Sequence:- " << sequence << std::endl;
	std::cout << "Latitude = " << Latitude << std::endl;
	std::cout << "Longitude = " << Longitude << std::endl;
	std::cout << endl << "int - " << sizeof(int) << " double - " << sizeof(double) << " long double" << sizeof(long double);
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
	  ros::init(argc, argv, "gps_handler_node");

	  /**
	    * NodeHandle is the main access point to communications with the ROS system.
	    * The first NodeHandle constructed will fully initialize this node, and the last
	    * NodeHandle destructed will close down the node.
	    */
	  ros::NodeHandle n;

	  // /fix is the topic on which the GPS unit publishes longitude and latitude
	  // We are creating a subscriber which can hear to messages
	  ros::Subscriber subscriber_for_fix = n.subscribe("/fix", 1, gpsLocationCallback);


	  /**
	    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	    * callbacks will be called from within this thread (the main one).  ros::spin()
	    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	    */
	  ros::spinOnce();
	  ofstream gpsFile;
	  char keyboardInput;
	  gpsFile.open ("gpsPointsTestPrecision.txt");
	  int counter = 1;
	  while (ros::ok())
	  {
		  std::cout << "Please press 'a' to store the current GPS position and 'd' to exit";
		  std::cin >> keyboardInput;
		  std::setprecision(15);
		  if (STORE_GPS)
		  {
			  ros::spinOnce();
			  gpsFile << "Seq number: " << counter << "\n";
			  gpsFile << std::setprecision(15) << "Latitude - " << Latitude << "    Longitude - " << Longitude << endl;
			  counter++;
		  }
		  else if (DONE_STORING_GPS)
		  {
			  gpsFile.close();
			  break;
		  }
	  }
}
