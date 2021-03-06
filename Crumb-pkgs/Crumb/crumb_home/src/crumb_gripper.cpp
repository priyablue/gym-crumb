#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include "geometry_msgs/Twist.h"


using namespace std;

/**
 * Basic teleoperation for WidowX arm.
 * Marina Aguilar-Moreno, 2016
 * 
 * DISCLAIMER: This is a work-in-progress test code, not finished yet.
 * 
 * This code must be integrated into a ros package, and then called with rosrun,
 * while gazebo is running.
 */
 
 
 

int main(int argc, char **argv)
{
  
  ros::init(argc, argv , "crumb_gripper");
  ros::NodeHandle n;
    

  ros::Publisher joint7 = n.advertise<std_msgs::Float64>("/gripper_1_joint/command", 1000);
  ros::Rate loop_rate(5);

  /**
   * Main code.
   */


  std_msgs::Float64 pinza;
  pinza.data = 0;
  int count = 0;
  char car='0';

  ROS_INFO("z: open gripper, x: close gripper\n");
  while (ros::ok() && count<=2000) 
  {

    scanf("%c",&car);    
    switch (car)
    {
		case 'z':
		{
			pinza.data =0.03;
			break;
		}
		case 'x':
		{
			pinza.data =0; 
			break;
		}								
	}

	joint7.publish(pinza);
	count++;	
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
