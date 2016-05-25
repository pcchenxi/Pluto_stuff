#ifndef ATRVNODE_H
#define ATRVNODE_H
#include <atrv/atrv.h>
#include <string>
#include <unistd.h>
#include <ros/ros.h>
//#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

/**
 *  \brief ATRV Node for ROS
 * By John Folkesson based on
 *  By Mikhail Medvedev 02/2012
 *  Modified from B21 node originally written by David Lu
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
class ATRVNode: public ATRV {
 public:
private:
  /**
   * Subscriber handles (cmd_vel,cmd_accel,cmd_sonar_power,cmd_brake_power)
   */
  ros::Subscriber subs[4];	       	
  /**
   * Sonar Publisher for Base Sonars (sonar_cloud_base) 
   */
  ros::Publisher sonar_pub;
  /**
   * Voltage Publisher (voltage)
   */
  ros::Publisher voltage_pub; 
  /**
   * Brake Power Publisher (brake_power)
   */
  ros::Publisher brake_power_pub;
  /**
   * Sonar Power Publisher (sonar_power)
   */
  ros::Publisher sonar_power_pub;
  /**
   * Odometry Publisher (odom)
   */
  ros::Publisher odom_pub;
  /**
   * Plugged In Publisher (plugged_in)
   */
  ros::Publisher plugged_pub;
  /**
   * Bump Publisher (bumps)
   */
  ros::Publisher bump_pub;
  /**
   * Transform Broadcaster (for odom)
   */
  tf::TransformBroadcaster broadcaster;
  unsigned char prev_bumps;
  
  ros::Time publishedTime;

  void publishOdometry();
  void publishSonar();
  void publishBumps();  
public:
  void publish(bool s);
  ros::NodeHandle n;

  ATRVNode();
  ~ATRVNode();
  int init(const char* port);
  /** 
   * Gets a point cloud for sonar readings from body 
   * param cloud structure into which the sonar readings are saved
   */
  void getSonarPoints(sensor_msgs::PointCloud* cloud);

  int getBumps(sensor_msgs::PointCloud* cloud);
  //  void spinOnce();

  // Message Listeners
  void NewCommand      (const geometry_msgs::Twist::ConstPtr& msg);
  void SetAcceleration (const std_msgs::Float32   ::ConstPtr& msg);
  void ToggleSonarPower(const std_msgs::Bool      ::ConstPtr& msg);
  void ToggleBrakePower(const std_msgs::Bool      ::ConstPtr& msg);
};

#endif
