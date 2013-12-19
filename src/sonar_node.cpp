/*
 *  Sonar node:- code for Turning ON/OFF the Sonars 
 *  Since the Sonars make the roscore to crash/hang the idea is to turn ON/OFF the
 *  Sonars at a fast rate and not keep it in one state for very long..
 *  Pulisher   : To Turn ON/OFF the Sonars on the atrvjr node
 *  Keval Patel - November 22, 2013
 *
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar_node");
  ros::NodeHandle n;

  ros::Publisher sonar_pub = n.advertise<std_msgs::Bool>("/atrvjr/cmd_sonar_power", 10);
  ros::Rate loop_rate(10);
  std_msgs::Bool sonarOnOffMessage;

  while (ros::ok())
  {
    // 1. Turn On the sonars by publishing true to the relevant topic
    // 2. Delay for some time (Need to test with different delay timings)
    // 3. Turn Off the sonars by publishing false to the relevant topic
    // 4. Delay for some time (Again need to test with different delay timings)
    // 5. Go back to step 1 and do this indefinitely   
   
    // Step 1:
    cout << endl << endl << "Turning the Sonars ON" << endl;
    sonarOnOffMessage.data = true;
    sonar_pub.publish(sonarOnOffMessage);
  
    // Step 2:
    cout << "Giving a short ON time delay" << endl;
    sleep(5);
  
    // Step 3:
    cout << "Turning the Sonars OFF" << endl;
    sonarOnOffMessage.data = false;
    sonar_pub.publish(sonarOnOffMessage);

    // Step 4:
    cout << "Giving a short OFF time delay" << endl;
    sleep(5);
  
    // Not sure what this does..
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

