#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "closestfunctions.h"
#include "kbquiting.h"
#include "closeststruct.h"

void scannerCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  scanner=*scan; //pointer to scan
  //Closest closestObject;
  closestObject=setClosestObject(scanner, closestObject); //updates the values of the closest object
  //ROS_INFO("The closest object is on the distance:  [%f] \n", closestObject.distance ); //prints the distance to the closest object
  //ROS_INFO("It has the angle: [%f] \n", closestObject.angle); //prints the angle to the closest object
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PLUTO_control");
  ros::NodeHandle n;
  
  //Suscribe to /scan
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scannerCallback);

  //Publish to cmd_vel
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  ros::Rate loop_rate(10);//10Hz (0.1s)
  
  bool y=true;
  std::cout << "Press Enter if you wish to quit."<< std::endl;
  while(ros::ok() and y)
  {
    geometry_msgs::Twist twister;
    twister=reactToObject(closestObject); //set vel and rot in twister
    pub.publish(twister); //publish speed

    //ros::Duration(0.1).sleep(); //sleep for 0.1s
    //stop moving (in case  of errors casing ROS to stop running before goal is reached.)
    //twister.linear.x=0;
    //twister.angular.z=0;
    //pub.publish(twister);

    //check if user has pressed enter (wish to quit)
    y=kbQuit();
    if (!y) //when the user has choosen to quit kbQuit returns false, set speed to zero before.
    {
      twister.linear.x=0;
      twister.angular.z=0;
      pub.publish(twister);
    }

    ros::spinOnce(); //allows a paus for subscriber callback before next loop
    loop_rate.sleep();
  }
  return 0;
}