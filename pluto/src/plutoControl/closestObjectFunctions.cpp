#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <sstream>
#include <std_msgs/Float32.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "closestfunctions.h"


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
  double dist_min=0.2;
  double dist_max=0.5;
  double k_rot; //rotational constant (0-1)
  double k_trans; //translational constant (0-1)
  double angle_diff;
  double dist_diff;
  //constant determining the impact of rot on vel. Bigger constant -> bigger impact.
  //no impact if 0, must be positive
  //maximal reduction (rot=pi/2): (0.2)^k
  //minimal reduction (rot=0.02): (0.98)^k
  double vel_const=5; 

  //Rotational speed; negative if the object is to the right
  //and positive if the object is to the left.
  //The sign is determined in angle_diff above.

  if(closestObject.angle==0) //either if the object is on the scanner or if the scanner doesn't publish values
  {
      angle_diff=1; //0 when recieving scan
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
  //k_rot to ensure that the "rotational breaking distance" is less than angle_error.
  //the formula d_stop=vt/2 has been used where t was approxed to 1, i.e. it was assumed
  //that the vehicle will stop in 1 sec.
  //k_rot=1000*abs(2*angle_error/angle_diff); //does not work...
  k_rot=0.25;
  double rot=angle_diff/0.1*k_rot; //v=s*k

  //Determine the present sign of the error in distance,
  //sets the difference accordingly.
  //(sign: if Pluto is to close, to far or in the allowed zone.)
  if(closestObject.distance==0) //either if the object is on the scanner or if the scanner doesn't publish values
  {
    dist_diff=3; //0 when recieving scan
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
  //towards the object is very big (Pluto needs to turn alot).

  //k_trans is set to ensure that the breaking distance is less than half of the allowed interval.
  //the formula dstop=vt/2 was used, t apporxed to 1.
  k_trans=abs((dist_max-dist_min)/dist_diff);
  double vel=dist_diff*k_trans/0.1*exp(-vel_const*abs(angle_diff));  //v=s*k_2*exp(-k_1*v_rot) (somewhere between s*k_2 and 0)

  //Set and return speed.
  twist.linear.x=vel;
  twist.angular.z=rot;
  return twist;
}
