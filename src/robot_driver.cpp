#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"

// Making this global is not required. There must be a safer way to do it.
// But for testing purposes I am initializing it here. To be removed later
double Distance = 0;
void xyDistanceCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	Distance = (*msg).data;
}
double Heading = 0;
void xyHeadingCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	Heading = (*msg).data;
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
  ros::Subscriber subscriber_for_gpsdistance;
  ros::Subscriber subscriber_for_gpsheading;
public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // set up the subscriber for the gpstoxy topic
    subscriber_for_gpsdistance = nh_.subscribe("/gpsdistance",1, xyDistanceCallBack);
    subscriber_for_gpsheading = nh_.subscribe("/gpsheading",1, xyHeadingCallBack);
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


  bool turnOdom(bool clockwise, double radians)
  {
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

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
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.2;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
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
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
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

  bool driveForward_safely(double distance)
  {
    if (distance > 2)
    {
      double d = distance - 2;
      driveForwardOdom(d,1);
      driveForwardOdom(0.5,0.8);
      driveForwardOdom(0.5,0.6);
      driveForwardOdom(0.5,0.4);
      driveForwardOdom(0.5,0.2);
    }
    else
    {
      driveForwardOdom(distance, 0.2); 
    }
  return true;
  }

  bool turnOdom_adjusted(double angle) //given an angle in radians, sends left/right turn message to motor function
  {
    double turningAngle = 0.0;

    if ((angle < -0.05) || (angle > 0.05)) //if we are not within +- .05 radians (~3 degrees) then we need to turn
    {
      if (Heading < 0) //if angle is negative, this is a counter-clockwise (left) turn.
      {
        turningAngle = -1 * angle; //make the angle positive
	turnOdom(false, turningAngle); //send the left turn command to the motors
	std::cout << std::setprecision(12) << "L: " << turningAngle * 180 / M_PI << std::endl;
      }
      else //this is a clockwise (right) turn
      {
	turningAngle = angle;
	turnOdom(true, turningAngle);
	std::cout << std::setprecision(12) << "R: " << turningAngle * 180 / M_PI << std::endl;
      }
    }
    else
    {
      std::cout << "No Turn" << std::endl;
    }

    return true;
  }

  bool drive(double distance)
  {
    int go_forward = 0;
    if (distance >= 40){ go_forward = 10; } 
    else if (distance >= 30) { go_forward = 8; }
    else if (distance >= 20) { go_forward = 6; }
    else if (distance >= 10) { go_forward = 4; }
    else if (distance >= 5) { go_forward = 2; }
    else { go_forward = 1; }

    driveForward_safely(go_forward);

    std::cout << std::setprecision(12) << "Dist: " << distance << " Drive: " << go_forward;
    return true;
  }

  bool turn(double angle)
  {
    double v_A, v_B, v_C, v_X, v_Y, v_Z, v_min, v_h1, v_h2;
    double avg_heading = 0;

    //2oo3 voter system
    sleep(2); //wait for sensors
    ros::spinOnce(); //get new data
    v_A = Heading; //get first heading

    sleep(2);
    ros::spinOnce();
    v_B = Heading; //get second heading

    sleep(2);
    ros::spinOnce();
    v_C = Heading; //get third heading
    /*
    Assume that the pair of headings which are the closest to one another are the best readings.
    If we take the absolute difference of each pair, the pair which produces the min were closest to one another 
    */
    v_X = abs(v_A - v_B); //take the absolute differences
    v_Y = abs(v_A - v_C); 
    v_Z = abs(v_B - v_C);
	
    v_min = v_X; //determine which difference was the smallest and save their components
    v_h1 = v_A;
    v_h2 = v_B;


    if (v_Y < v_min)
    {
      v_min = v_Y;
      v_h1 = v_A;
      v_h2 = v_C;
    }
	
    if (v_Z < v_min)
    {
      v_min = v_Z;
      v_h1 = v_B;
      v_h2 = v_C;
    }

    avg_heading = (v_h1 + v_h2) / 2.0; //average the two best readings for an EVEN BETTER READING

    std::cout << std::setprecision(3) << "{ " << v_A * 180/M_PI << ", " << v_B * 180/M_PI << ", " << v_C * 180/M_PI << " }" <<std::endl;
  
    turnOdom_adjusted(avg_heading);

    return true;

  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver(nh);
  //int avg_iterations = 5; //number of headings to average before turn


  ros::spinOnce();
  while (Distance == 0)
  {
  	ros::spinOnce();
  }

  do
  {
	driver.turn(Heading); //turn
	driver.drive(Distance); //drive	
	ros::spinOnce(); //get new data

  }while (Distance > 3.5); //check whether we are within 6m of our target


std::cout << "Destination has been reached" << std::endl;
std::cout << std::setprecision(12) << "Estimated Distance from Target: " << Distance << std::endl;
sleep(5);
ros::spinOnce();
driver.turn(Heading);

return 0;
//  std::cout << std::endl << "Enter the value to move forward" << std::endl;
//  std::cin >> forwardDistance;
// driver.driveForwardOdom(Distance);
//  std::cout << std::endl << "Enter the turning angle" << std::endl;
//  std::cin >> turningAngle;
//  driver.turnOdom(true,turningAngle);
}
