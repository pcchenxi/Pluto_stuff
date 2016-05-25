#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Path.h>

using namespace std;

bool scan_finish            = false;
bool scan_start             = false;
bool local_motion_finish    = false;

bool mission_finish         = false;
bool mission_start          = false;

bool goal_sent              = false;

bool costmap_updated        = false;

bool globalpath_updated     = false;

bool need_stop		    	= false;

bool final_goal_updated	    = false;
bool local_goal_updated	    = false;
bool stoppose_calculate_sent= false;

bool check_finish	    = false;

double global_path_time      = 0;

float robot_x_current;
float robot_y_current;
float robot_x_start;
float robot_y_start;

float stop_x;
float stop_y;

float robot_tilt;
float robot_pan;

ros::Publisher  pub_startscan, goal_pub, local_cmd_pub, pub_scan, pub_pluto_path;

geometry_msgs::PoseStamped final_goal, local_goal;
nav_msgs::Path global_path;

