/*
*Subscribes to /scan and calculates and saves the distance and angle to teh closest object.
*Publish speed to cmd_vel to move towards the closest object. That is to have a distance of 0-5-0.6m to it,
*and to be directed towards it within an angular error of 0.02rad.
*/


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <sstream>
#include <std_msgs/Float32.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::LaserScan scanner;

/* scanner members below
*scanner.ranges[i]=distance to closest object in angle_i, i.e. i=0 - angel=angle_min
*scanner.angle_min = min angle (rad)
*scanner.angle_max = max angle (rad)
*scanner.angle_increment= angle steps (difference between each measurement)
*scanner.time_increment = time between each measurement
*scanner.scan_time=?
*scanner.range_min= min range for measurement
*scanner.range_max= max range the scanner can detect
*/

struct Closest //structure of the closest object
{
  float distance;
  float angle;
  int index;
};

Closest closestObject;

int lengthRangesInt(sensor_msgs::LaserScan scanner) //calulates the number of values in ranges
{
  float lengthRange=(scanner.angle_max-scanner.angle_min)/scanner.angle_increment;
  int lengthRangeInt=static_cast<int>(round(lengthRange));
  return lengthRangeInt;
}

//Sets the values of closestObject. Initially the values of index and distance are given as the first element in ranges.
//The values are then updated if other elements are smaller than the first in the for/if loop.
Closest setClosestObject(sensor_msgs::LaserScan scanner, Closest closestObject) 
{
  int lengthRange=lengthRangesInt(scanner);
  float smallest=scanner.ranges[0];
  int index=0;
  for (int i=1; i<lengthRange+1; i++)
  {
    if (smallest>scanner.ranges[i])
    {
      smallest=scanner.ranges[i];
      index=i;
    }
  }
  closestObject.distance=smallest;
  closestObject.index=index;
  //Angle is calculated using the index
  closestObject.angle=scanner.angle_min+(index*scanner.angle_increment);
  return closestObject;
}

geometry_msgs::Twist reactToObject(Closest closestObject)//publish speed to cmd_vel if not within given limits
{
 
  geometry_msgs::Twist twist;
  
  double angle_error=0.1;
  double dist_min=0.7;
  double dist_max=1;
  double angle_diff;
  double dist_diff;
  double vel_const;

  //Rotational speed; negative if the object is to the right
  //and positive if the object is to the left.
  //The sign is determined in angle_diff above.

  if(closestObject.angle==0) //either if the object is on the scanner or if the scanner doesn't publish values
  {
      angle_diff=0;
  }
  else if(closestObject.angle < -angle_error)
  {
    angle_diff=closestObject.angle+angle_error;
  }
  else if(closestObject.angle > angle_error)
  {
    angle_diff=closestObject.angle-angle_error;
  }
  else
  {
    angle_diff=0;
  }

  double rot=angle_diff; //divide by 0.1 for maximum speed (wrt velocity not acceleration!)

  //Determine the present sign of the error in distance,
  //sets the difference accordingly.
  //(sign: if Pluto is to close, to far or in the allowed zone.)
  if(closestObject.distance==0) //either if the object is on the scanner or if the scanner doesn't publish values
  {
    dist_diff=0;
  }
  else if(closestObject.distance < dist_min)
  {
    dist_diff=closestObject.distance-dist_min;
  }
  else if(closestObject.distance > dist_max)
  {
    dist_diff=closestObject.distance-dist_max;
  }
  else
  {
    dist_diff=0;
  }
  //Sets the translational speed.
  //Negative if the object is too close, 
  //positive if the object is too far
  //zero if either the object is in the zone or if the angle
  //towards the object is very big.
  double vel=dist_diff*exp(-vel_const*abs(angle_diff));  //divide by 0.1 for maximum speed (wrt velocity not acceleration!)

  //Set and return speed.
  twist.linear.x=vel;
  twist.angular.z=rot;
  return twist;
}

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
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  
  //Suscribe to /scan
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scannerCallback);

  //Publish to cmd_vel
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  ros::Rate loop_rate(10);//10Hz (0.1s)
  
  while(ros::ok())
  {
    geometry_msgs::Twist twister;
    twister=reactToObject(closestObject);
    pub.publish(twister);
    //ros::Duration(0.1).sleep(); //sleep for 0.1s
    //stop moving (in case  of errors casing ROS to stop running before goal is reached.)
    //twister.linear.x=0;
    //twister.angular.z=0;
    //pub.publish(twister);
    //if user press enter: stop and quit

    ros::spinOnce(); //allows a paus for subscriber callback before next loop
    loop_rate.sleep();
  }
  return 0;
}