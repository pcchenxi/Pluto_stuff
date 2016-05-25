#include <cstdio>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Vector3.h"

#include "sensor_msgs/LaserScan.h"

#include <tf/transform_listener.h>

using namespace std;

geometry_msgs::Vector3 flag;

ros::Publisher  pub, pub_start, pub_end;
ros::Publisher  pub_assembed;
ros::Publisher  pub_scan_selected;
ros::Publisher  odom_pub;

int   angle_motion_number = 6;
float tilt_angle_array[9]  = {0.0, -0.5, -0.5,   0.0,   0.0,  -0.5,  0.0,  -0.0,  0.3};
float pan_angle_array[9]   = {0,    0,   -0.5,  -0.5,   0.5,   0.5,  0.0,   0.0,  0.0};
//
//int   angle_motion_number = 9;
//float tilt_angle_array[9]  = {0.0, -0.07,  -0.14, -0.21,  -0.28,  -0.35,  -0.42, -0.49, -0.56};
//float pan_angle_array[9]   = {0,    0,      0,    0,       0,      0,      0,     0,     0};

//int   angle_motion_number = 150;
//float tilt_angle_array[150];
//float pan_angle_array[150];
//float tilt_angle_array[6]  = {0,  -0.2,   0,  0.24,   0.22,  0.0};
//float pan_angle_array[6]   = {0,    0,      0,     0,     0,     0};

int   angle_index = 0;

int   move_count = 0;
bool  init_success = false;
bool  moving_pluto = false;
bool  first_move = true;



bool  tilt_stable = false;
bool  read_success = false;
sensor_msgs::LaserScan scan_cur;
//sensor_msgs::LaserScan scan_data;

void move_pluto(float time)
{
    cout << "moving robot" << endl;
    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;
   // odom_pub.publish(twist);

    ros::Duration(5).sleep();

    twist.linear.x = 0;
  //  odom_pub.publish(twist);

    angle_index = 0;
    init_success = false;
  //  cout << "finish moving robot" << endl;
}

void initTiltPose(float tilt, float pan)
{
    geometry_msgs::Vector3 tiltPose;
    tiltPose.x = pan;
    tiltPose.y = tilt;
    tiltPose.z = 0;

    pub.publish(tiltPose);
    //cout << "    commend send  " <<  tilt <<"   " << pan <<endl;
    //ros::Duration(0.2).sleep();
}

void callback_pantilt(const geometry_msgs::Vector3::ConstPtr &msgs)
{
    float tilt_read   = msgs->y;
    float pan_read  = msgs->x;

    float tilt_set      = tilt_angle_array[angle_index] ;
    float pan_set    = pan_angle_array[angle_index] ;

    float diff_tilt     = abs(tilt_set - tilt_read);
    float diff_pan   = abs(pan_set- pan_read);

    if(diff_tilt < 0.01&& diff_pan<0.01)
    {
        ros::Duration(0.4).sleep();
        tilt_stable = true;
        // start to record
        if(angle_index == 0)
        {
          init_success = true;
          pub_start.publish(flag);
          //cout <<"init_success  " << init_success<<endl;
        }

        angle_index ++;

        if(angle_index == 1 || angle_index == 3 || angle_index == 5)
        {
            pub_start.publish(flag);
        }

        if(angle_index == 2 || angle_index == 4)
        {
            flag.x = 0.0;
            pub_end.publish(flag);
        }

        cout << angle_index;
        if(angle_index == angle_motion_number)
          return;
        else
        {
          cout<< "tilt: " << tilt_angle_array[angle_index] << "  index: " << angle_index << endl;
          initTiltPose(tilt_angle_array[angle_index], pan_angle_array[angle_index]);
          //pub_scan_record.publish(scan_data);
        }
    }
    else
    {
      ros::Duration(0.2).sleep();

      //pub_scan_selected.publish(scan_cur);
      //initTiltPose(tilt_angle_array[angle_index], pan_angle_array[angle_index]);
    }
}

void callback_scan (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(tilt_stable)
    {
        pub_scan_selected.publish(*scan_in);
        cout << "scan " << endl;
        tilt_stable = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pluto_ptu_scan");
    ros::NodeHandle n;
    ros::Publisher joint_pub;
    ros::Rate loop_rate(100);

    pub = n.advertise<geometry_msgs::Vector3>("/ptu/cmd_pos", 10);

    ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, callback_scan);
    ros::Subscriber sub_pantilt = n.subscribe<geometry_msgs::Vector3>("/ptu/ptupos", 1, callback_pantilt);

 //   pub_assembed = n.advertise<sensor_msgs::PointCloud2> ("assembled_cloud", 1);
    odom_pub = n.advertise<geometry_msgs::Twist>("/atrv/cmd_vel", 50);

    pub_start = n.advertise<geometry_msgs::Vector3> ("start_flat", 1);
    pub_end = n.advertise<geometry_msgs::Vector3> ("end_flat", 1);
    pub_scan_selected = n.advertise<sensor_msgs::LaserScan> ("scan2", 1);

//    float increase = 0.013;
//    float start = 0;
//    float finish = -0.4;
//    for(int i = start; i<100; i++)
//    {
//        float angle = start - increase*i;
//        tilt_angle_array[i] = angle;
//        pan_angle_array[i] = 0;
//
//        if(angle < finish)
//        {
//            angle_motion_number = i;
//            cout << "number" << angle_motion_number << endl;
//            break;
//        }
//    }
//
//    //tilt_angle_array[i] = angle;
//    float start_angle = tilt_angle_array[angle_motion_number-1];
//cout << "start_angle" << start_angle << endl;
//    pan_angle_array[angle_motion_number] = -0.3;
//
//    for(int i = angle_motion_number; i<200; i++)
//    {
//        float angle = start_angle + increase*(i - angle_motion_number);
//
//        tilt_angle_array[i] = angle;
//        pan_angle_array[i] = -0.3;
//
//        if(angle >0)
//        {
//            angle_motion_number = i;
//            cout << "number" << angle_motion_number << endl;
//            break;
//        }
//    }



//    start_angle = tilt_angle_array[angle_motion_number-1];
//    cout << "start_angle" << start_angle << endl;
//    pan_angle_array[angle_motion_number] = 0.3;
//
//    for(int i = angle_motion_number; i<200; i++)
//    {
//        float angle = start_angle - increase*(i - angle_motion_number);
//
//        tilt_angle_array[i] = angle;
//        pan_angle_array[i] = 0.3;
//
//        if(angle < finish)
//        {
//            angle_motion_number = i;
//            cout << "number" << angle_motion_number << endl;
//            break;
//        }
//    }



    while (ros::ok() )
    {
        if(angle_index == angle_motion_number)
        {
            flag.x = 1.0;
            pub_end.publish(flag);
            angle_index = 0;
            init_success = false;
            cout << "finish" << endl;
 //           move_pluto(1);
 //           break;
        }

        if(move_count == 2)
        {
            break;
        }

        if(init_success == false)
        {
           initTiltPose(tilt_angle_array[0], pan_angle_array[0]);
           cout << "waiting for init  " << init_success<<endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    pub_end.publish(flag);
    initTiltPose(0, 0);
    return 0;
}
