#include <atrv/atrvnode.h>


ATRVNode::ATRVNode() : n ("~") {


  prev_bumps = 1;


  subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,   &ATRVNode::NewCommand, this);
  subs[1] = n.subscribe<std_msgs::Float32>("cmd_accel", 1,     &ATRVNode::SetAcceleration, this);
  subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &ATRVNode::ToggleSonarPower, this);
  subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &ATRVNode::ToggleBrakePower, this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud", 50);
  sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
  brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
  voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
  plugged_pub = n.advertise<std_msgs::Bool>("plugged_in", 1);
  bump_pub = n.advertise<sensor_msgs::PointCloud>("bump", 5);
}
int ATRVNode::init(const char* port) {
    int ret = initialize(port);
    if (ret < 0)
        return ret;
    return 0;
}
ATRVNode::~ATRVNode() {
  motionSetDefaults();
  setOdometryPeriod(0);
  setDigitalIoPeriod(0);
  setSonarPower(false);

}

#include <iostream>
#include <unistd.h>
#include <sys/time.h>
//global variable timer
struct timeval last_call, time_now;

/// cmd_vel callback
void ATRVNode::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
  //com.setVelocity(msg->linear.x, msg->angular.z);
  gettimeofday(&last_call, NULL); //update current time
  com.setVelocity(msg->linear.x, msg->angular.z);
}

/// cmd_acceleration callback
void ATRVNode::SetAcceleration (const std_msgs::Float32::ConstPtr& msg) {
  com.setAcceleration(msg->data);
}

/// cmd_sonar_power callback
void ATRVNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
  bool i=(bool)msg->data;
  //Before: com.setSonarPower(i);
  setSonarPower(i);
}

/// cmd_brake_power callback
void ATRVNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    bool i=msg->data;
    setBrakePower(i);
}

void ATRVNode::getSonarPoints(sensor_msgs::PointCloud* cloud) {
    float readings[12];
    getSonarReadings( readings);
    cloud->points.resize(12);
    for (int i = 0; i < 12; ++i) {
      if (readings[i] < (double)sonarMaxRange/(double)sonarRangeConversion) {
	  double angle =  SONARS_THETA[i];
            angle *= M_PI / 180.0;
            cloud->points[i].x = SONARS_X[i] + readings[i]*cos(angle);
            cloud->points[i].y = SONARS_Y[i] + readings[i]*sin(angle);
            cloud->points[i].z = SONARS_Z[i];
        }
    }
}

int ATRVNode::getBumps(sensor_msgs::PointCloud* cloud) {
  int total = 0;
  int value = parserState.bump;
  int mask = 1;
  for (int j=0;j<BUMPER_COUNT;j++) {
    if ((value & mask) > 0) {
	total++;
    }
    mask = mask << 1;
  }
  cloud->points.resize(total);
  if (total==0)
    return 0;
  int i=0;
  mask=1;
  for( int j=0; j< BUMPER_COUNT; j++){
    if ((value & mask) > 0) {
      cloud->points[i].x = BUMPER_X[j];
      cloud->points[i].y = BUMPER_Y[j];
      cloud->points[i].z = BUMPER_Z[j];
      i++;
    }
    mask = mask << 1;
  }
  return value;
}
void ATRVNode::publish(bool s) {
  //hard code teh ros publish rate to =10_- Hz
  ros::Duration dur=parserState.packetTime-publishedTime;
  //if (dur.toSec()<0.1)return; //TODO <- THIS MIGHT BE THE THING CAUSING TROUBLE!
  publishedTime=parserState.packetTime;
  std_msgs::Bool bmsg;
  publishOdometry();
  if (useSonar){
    bmsg.data = s;
    sonar_power_pub.publish(bmsg);
    publishSonar();
  }
  if (useDIO){
    bmsg.data = getBrakePower();
    brake_power_pub.publish(bmsg);
    bmsg.data = isPluggedIn();
    plugged_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = parserState.voltage;
    voltage_pub.publish(vmsg);
    publishBumps();
  }
}

/** Integrates over the lastest raw odometry readings from
 * the driver to get x, y and theta */
void ATRVNode::publishOdometry() {
    if (!isOdomReady()) {
        return;
    }
    float bearing = angles::normalize_angle(parserState.theta);
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(bearing);
    geometry_msgs::Quaternion odom_quat_pose = tf::createQuaternionMsgFromYaw(bearing);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = parserState.odoTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = parserState.x;
    odom_trans.transform.translation.y = parserState.y;
    odom_trans.transform.translation.z = 0.19;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);
     //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = parserState.odoTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = parserState.x;
    odom.pose.pose.position.y = parserState.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat_pose;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = parserState.v*cos(bearing);
    odom.twist.twist.linear.y = parserState.v*sin(bearing);
    odom.twist.twist.angular.z =parserState.omega;
    //publish the message
    odom_pub.publish(odom);
    std::cout << parserState.odoTime << "publish odom x: " << odom_trans.transform.translation.x << " y: " << odom_trans.transform.translation.y << std::endl;
}

void ATRVNode::publishSonar() {
   if (newSonar) {
     sensor_msgs::PointCloud cloud;
     cloud.header.stamp = parserState.sonarTime;
     cloud.header.frame_id = "base_link";
     getSonarPoints(&cloud);
     sonar_pub.publish(cloud);
     newSonar=0;
    }
}

void ATRVNode::publishBumps() {
    sensor_msgs::PointCloud cloud1;
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "base_link";
    int bumps = getBumps(&cloud1);
    if (bumps>0 || prev_bumps>0) {
        bump_pub.publish(cloud1);
    }
    prev_bumps = bumps;
}

#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <fstream>

bool notdone=true;
pthread_mutex_t signal_mutex;
int signal_val;
void sigint_handler(int x);
void sigtrap(int) { notdone = false; }

int main(int argc, char** argv) {
  gettimeofday(&last_call, NULL);  //set first current value
  const char *optstring = "h:p:c:";
  std::string args = "[-h help] [-c not implemented] [-p serialport] [-d debug level] ";
  std::string portstring="/dev/ttyUSB1";
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 'p':
      portstring = optarg;
      break;
    case 'c':
      break;
    case 'h':
    case '?':
      std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
      std::cerr << "Example " << argv[0] << " "
		<<"-p /dev/tty0"<<std::endl;
      return -1;
    }
    o = getopt(argc, argv, optstring);
  }
    ros::init(argc, argv, "atrv");
    ATRVNode node;

    std::string port;
    //    node.n.param<std::string>("port", port, "/dev/ttyUSB0");
    node.n.param<std::string>("port", port, portstring.c_str());
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node.init(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
        return 0;
    }
    ROS_INFO("Connected!");

      //MAIN LOOP
     //notdone = true;
      bool stopped = false;
     ros::Rate r(20);
     long ms_limit=1000; //time limit in ms which is allowed to pass between updates
     long time_past;
     signal(SIGINT, sigint_handler); //set sigint
     while (notdone)
     {
     	gettimeofday(&time_now, NULL);
     	time_past=(time_now.tv_sec-last_call.tv_sec)*1000+(time_now.tv_usec-last_call.tv_usec)/1000;

        if(time_past>ms_limit)
        {
            if (stopped==false){
                printf("Stopped because of inaction\n");
                node.com.setVelocity(0,0);
                stopped = true;
            }
        }
        else stopped = false;
        node.main_update();

      	ros::spinOnce();
      	r.sleep();
     }
     //Here is where you end up if you press ctl-c
     ROS_INFO("The user interrupted the program. ");
     node.com.setVelocity(0, 0); //Stahp Pluto! Stahp!
    exit(0);
    return 0;

}


void sigint_handler(int a)
{
  //signal(SIGINT, sigint_handler);
  //if(signal_val == 1) {
    //return;
  //}
  //signal_val = 1;
  fprintf(stderr,
	  "Caught signal %i (%s)\n",
	  a, a==2?"interrupt": a==1? "terminate" : a==15? "killall" :
	  a==9? "die you dog": a==12? "go next" :  //Erik's comment: I'm not responsible for this line of code!
          a==14? "alarm timeout, exiting": "unknown");
  if (a==12){
    std::cerr<<"got the go next signal\n";
  }
  else {
  	//exit(a); //exit the tread which was interrupted
  	notdone=false;
  }
  pthread_mutex_unlock(&signal_mutex);
}
