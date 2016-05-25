#!/usr/bin/env python
import rosbag,rospy,datetime
from std_msgs.msg import Int32, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

'''
run together with joy_node and kinect2_bridge.
Press a on xbox-controller to start recording.
press b on xbox-controller to stop recording.
'''

kinectbag = None #rosbag.Bag('test.bag', 'w')
laserbag = None

recording_kinect = False
recording_laserscan = False
filenumber = 0
semafore_kinect = True
semafore_laser = True

kinect_counter = 0
laser_counter = 0

def kinect_callback(data):
	if recording_kinect and semafore_kinect:
		kinectbag.write('/kinect2/sd/points',data)

def laser_callback(data):
	if recording_laserscan and semafore_laser:
		laserbag.write('/scan',data)


def joy_callback(data):
	global kinectbag,laserbag,recording_laserscan,recording_kinect,semafore_kinect,semafore_laser,kinect_counter,laser_counter
	if data.buttons[0] and not recording_kinect:
		semafore_kinect = False
		recording_kinect = True
		i = datetime.datetime.now()
		kinectbag = rosbag.Bag("kinectscan"+str(kinect_counter)+"-"+i.isoformat()+"recording.bag",'w')
		print "started recording in: kinectscan"+str(kinect_counter)+"-"+i.isoformat()+"recording.bag"
                kinect_counter+=1
		semafore_kinect = True
	elif data.buttons[1] and recording_kinect:
		semafore_kinect = False
		recording_kinect = False
		print "stopped recording kinectscan"
		kinectbag.close()
		semafore_kinect = True
	elif data.buttons[2] and not recording_laserscan:
		semafore_laser = False
		recording_laserscan = True
		i = datetime.datetime.now()
		laserbag = rosbag.Bag("laserscan"+str(laser_counter)+"-"+i.isoformat()+"recording.bag",'w')
		print "started recording in: laserscan" +str(laser_counter)+"-"+i.isoformat()+"recording.bag"
		laser_counter+=1
		semafore_laser = True
	elif data.buttons[3] and recording_laserscan:
		semafore_laser = False
		recording_laserscan = False
		print "stopped recording laserscan"
		laserbag.close()
		semafore_laser = True






rospy.init_node('kinect_recorder', anonymous=True)
rospy.Subscriber("/kinect2/sd/points", PointCloud2, kinect_callback)
rospy.Subscriber("/joy",Joy,joy_callback)
rospy.Subscriber("/scan",LaserScan,laser_callback)
rospy.spin()


