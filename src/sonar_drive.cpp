#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"

double Sonar_Reading = 5;
void update_sonar_data(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  Sonar_Reading = (msg)->points[15].x;
}

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;
  //  We will be subscribing to distance and heading to be covered that is calculated by python script
  ros::Subscriber subscriber_for_sonar_data;
public:

  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    subscriber_for_sonar_data = nh_.subscribe("/atrvjr/sonar_cloud_body",1,update_sonar_data);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance, double speed)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("/base_link", "odom",
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("/base_link", "odom",
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = speed;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("/base_link", "odom",
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if(dist_moved > distance) done = true;
    }
    if (done) 
    {
	base_cmd.linear.y = base_cmd.angular.z = 0;
    	base_cmd.linear.x = 0.0;
    	//send the drive command
    	cmd_vel_pub_.publish(base_cmd);
    	rate.sleep();
	return true;
    } 	
    return false;
  }

};

bool sonar_safe()
{
  std::cout << Sonar_Reading <<std::endl;

  if (Sonar_Reading < 1.0)
  {
    return false;
  }
 
  return true;
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "sonar_drive");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
  ros::Publisher sonar_cmd_pub = nh.advertise<std_msgs::Bool>("/atrvjr/cmd_sonar_power", 10);

  std_msgs::Bool sonar_command;

  sonar_command.data = true; //turn on sonars
  sonar_cmd_pub.publish(sonar_command);
  std::cout << "Waiting 5 seconds" << std::endl;
  sleep(5);
  sonar_cmd_pub.publish(sonar_command);
  ros::spinOnce();
  do
  {
	driver.driveForwardOdom(1,0.5);
	ros::spinOnce(); //get new data

  }while (sonar_safe()); //check whether the sonars have detected an obstacle
  
sonar_command.data = false; //shut off sonars
sonar_cmd_pub.publish(sonar_command);
std::cout << "Sonars detected obstacle" << std::endl;

return 0;
}
