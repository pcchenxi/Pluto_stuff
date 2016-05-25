#include "pluto_action_planner.h"
#include <move_base_msgs/MoveBaseAction.h>

void callback_goalinput(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    if(!final_goal_updated)
    {
        cout << "final goal recieved" << endl;
        final_goal = *goal;
        mission_start = true;

        final_goal_updated = true;
        local_goal_updated  = false;
    }
    else
    {
        local_goal = *goal;
        local_goal_updated = true;
        global_path_time = ros::Time::now().toSec();
    }

    robot_x_start = robot_x_current;
    robot_y_start = robot_y_current;

    goal_sent = true;
    globalpath_updated = false;
}

void callback_odom_robot(const nav_msgs::Odometry::ConstPtr& odomsg)
{
    robot_x_current = odomsg->pose.pose.position.x;
    robot_y_current = odomsg->pose.pose.position.y;

//    if(!final_goal_updated || scan_start)
//        return;

//    else if(time_diff > 4)
//    {
//        local_motion_finish = true;
//        costmap_updated = false;
//        cout << "request to scan again" << endl;
//    }

    //odom_recived = true;
}

void callback_localcmd(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    if(!local_goal_updated || scan_start)
        return;

    //double time_now = ros::Time::now().toSec();
    //double time_gpath = global_path.header.stamp.toSec();

    //float time_diff = (time_now - global_path_time);
    //if(dist > 0.2 || !need_stop)
    //if(time_diff < 4)
    {
        local_cmd_pub.publish(*cmd_vel);
        cout << "move.. distance to local stop: " << endl;
    }

    //odom_recived = true;
}

void callback_scanfinish(const geometry_msgs::Vector3::ConstPtr &msgs)
{
    geometry_msgs::Vector3 flag = *msgs;
    if(flag.x == 1)
    {
        scan_finish = true;
        scan_start = false;
        cout << "scan finish" << endl;
        global_path_time = ros::Time::now().toSec();
    }

}

void callback_costmapupdate(const geometry_msgs::Vector3::ConstPtr &msgs)
{
    costmap_updated = true;
    if(final_goal_updated)
        goal_pub.publish(final_goal);
        
    cout <<"costmap_updated" << endl;
}

void callback_scan (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(abs(robot_tilt) < 0.01)
        pub_scan.publish(*scan_in);
}

void callback_pantilt(const geometry_msgs::Vector3::ConstPtr &msgs)
{
    robot_tilt = msgs->y;
    robot_pan  = msgs->x;
}

void callback_reachgoal(const move_base_msgs::MoveBaseActionResult::ConstPtr &result)
{
    if(result->status.status != 3)
        return;

    local_motion_finish = true;
    costmap_updated = false;

    float diff_x = abs(robot_x_current - final_goal.pose.position.x);
    float diff_y = abs(robot_y_current - final_goal.pose.position.y);

    float dist = sqrt(diff_x*diff_x + diff_y*diff_y);

    float diff_x_stop = abs(robot_x_current - stop_x);
    float diff_y_stop = abs(robot_y_current - stop_y);

    float dist_stop= sqrt(diff_x*diff_x + diff_y*diff_y);

    cout << "distance to target position: " << dist << endl;

    if(dist < 0.25)
    {
        mission_finish = true;
        mission_start = false;
        goal_sent = false;
        cout << "finish" << endl;
    }
    else
    {
        cout << "motion finish request to scan again" << endl;
        local_motion_finish = true;
        costmap_updated = false;
        scan_finish = false;
    }
}

void callback_globalplan(const nav_msgs::Path::ConstPtr &global_path_msgs)
{
    if(!mission_start)
        return;
    global_path = *global_path_msgs;
    //global_path_time = ros::Time::now().toSec();
    //cout << "global_path recieved " << global_path.poses.size() << endl;
    globalpath_updated = true;
}

void callback_stoppose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    geometry_msgs::PoseStamped g;
    if(pose->pose.position.x == 0 && pose->pose.position.y == 0)
    {
        g = final_goal;
        need_stop = false;
        cout << "no need to stop" << endl;
    }
    else
    {
        g = *pose;
        need_stop = true;
        cout << "need to stop" << g.pose.position.x << " " << g.pose.position.y << endl;
    }

    stop_x = g.pose.position.x;
    stop_y = g.pose.position.y;

    robot_x_start = robot_x_current;
    robot_y_start = robot_y_current;

    goal_pub.publish(g);
    cout << "stop point updated" << stop_x << "   " << stop_y << endl;
    cout << "sending the local goal." << endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pluto_action_planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // //scan
    // ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, callback_scan);
    // pub_scan = n.advertise<sensor_msgs::LaserScan>("/scan_gmapping", 1);
    // ros::Subscriber sub_pantilt = n.subscribe<geometry_msgs::Vector3>("/ptu/ptupos", 1, callback_pantilt);

    // // motion control
    // local_cmd_pub = n.advertise<geometry_msgs::Twist>("/atrv/cmd_vel", 50);
    // ros::Subscriber local_cmd_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 50, callback_localcmd);

    // // scan
    // pub_startscan = n.advertise<geometry_msgs::Vector3> ("start_scan", 1);
    // ros::Subscriber sub_scanfinish = n.subscribe<geometry_msgs::Vector3> ("end_flat", 1, callback_scanfinish);

    // goal
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, callback_goalinput);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Subscriber reach_goal_sub = n.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, callback_reachgoal);

    // // odom
    // ros::Subscriber sub_odom_robot = n.subscribe<nav_msgs::Odometry>("/atrv/odom", 1, callback_odom_robot);

    // // path plan
    // ros::Subscriber sub_globalpath = n.subscribe<nav_msgs::Path>("/move_base/DWAPlannerROS/global_plan", 1, callback_globalplan);
    // pub_pluto_path = n.advertise<nav_msgs::Path>("/pluto_actionplanner/global_plan", 1);
    // ros::Subscriber sub_stoppose = n.subscribe<geometry_msgs::PoseStamped>("/stop_pose", 1, callback_stoppose);

    ros::Subscriber sub_costmap_updated = n.subscribe<geometry_msgs::Vector3> ("costmap_updated", 1, callback_costmapupdate);

    geometry_msgs::Vector3 flag;
    int count = 0;
    int print = 0;
    while (ros::ok())
    {
        // count ++;
        // if(count > 10)
        //     count = 11;
        // if(mission_finish)
        // {
        //     cout << "finish " << endl;
        //     mission_start = false;
        //     scan_finish = false;
        //     final_goal_updated = false;
        //     local_goal_updated = false;
        //     stoppose_calculate_sent = false;
        //     costmap_updated = false;
        //     local_motion_finish = false;
        //     scan_start = false;
        //     mission_finish = false;
        //     stoppose_calculate_sent = false;

        //     count = 0;
        //    // break;
        // }

        // else if(!mission_start && !scan_finish && count < 5)   // first scan
        // {
        //     pub_startscan.publish(flag);

        //     if(count == 4)
        //     {
        //         scan_start = true;
        //         scan_finish = false;
        //         cout << "scanning " << endl;
        //     }
        // }

        // else if(costmap_updated && !mission_start)  // wait for user input final target
        // {
        //     if(print == 0)
        //         cout << "waiting for goal input " << endl;
        //     print = 1;
        // }

        // else if(final_goal_updated && costmap_updated && globalpath_updated && !local_goal_updated && !stoppose_calculate_sent) // calculate the stop point
        // {
        //     pub_pluto_path.publish(global_path);  // publish the stop point inside
        //     cout << "calculate the stop point" << endl;
        //     stoppose_calculate_sent = true;
        //     ros::Duration(1.0).sleep();
        // }
        // else if(local_motion_finish && !scan_start && !scan_finish)
        // {
        //     pub_startscan.publish(flag);
        //     ros::Duration(2.0).sleep();
        //     scan_start = true;
        //     scan_finish = false;
        //     goal_sent = false;
        //     local_goal_updated = false;
        //     final_goal_updated = false;
        //     costmap_updated = false;
        //     stoppose_calculate_sent = false;
        //     globalpath_updated = false;
        // }
        // else if(local_motion_finish && costmap_updated)
        // {
        //     goal_pub.publish(final_goal);
        //     local_motion_finish = false;

        //     cout << "sending the final goal" << endl;
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
