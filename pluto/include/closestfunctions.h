#ifndef CLOSESTFUNCTIONS_H
#define CLOSESTFUNCTIONS_H

struct Closest
{
  float distance;
  float angle;
  int index;
};

int lengthRangesInt(sensor_msgs::LaserScan scanner);

Closest setClosestObject(sensor_msgs::LaserScan scanner, Closest closestObject);

geometry_msgs::Twist reactToObject(Closest closestObject);

#endif