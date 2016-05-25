#!/usr/bin/env python

import struct,time,rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

#A simplified wrist program just for testing purposes



pan  = 0.0
tilt = 0.0

panpos = 0.0
tiltpos = 0.0

new_nessage = True
pub = rospy.Publisher("/ptu/cmd_pos",Vector3,queue_size = 10)

def joystick_callback(joymsg):
	
	global pan,tilt,panpos,tiltpos
	if abs(joymsg.axes[3])>0.12:
		pan = joymsg.axes[3]*-0.01
	else:
		pan = 0.0
	if abs(joymsg.axes[4])>0.12:
		tilt = joymsg.axes[4]*0.01
	else:
		tilt = 0.0
	if joymsg.buttons[5]:
		panpos = 0.0
		tiltpos = 0.0

	print 'new Message recieved'
	vector = Vector3()
	panpos  += pan
	tiltpos += tilt
	panpos = max(min(panpos,1.0),-1.0)
	tiltpos = max(min(tiltpos,0.3),-1.0)
	vector.x = panpos
	vector.y = tiltpos
	pub.publish(vector)
		

def main():
	global panpos,tiltpos
	rospy.init_node("joystick_ptu_node",anonymous = True)
	rospy.Subscriber("/joy",Joy,joystick_callback)
	pub = rospy.Publisher("/ptu/cmd_pos",Vector3,queue_size = 10)
	#rospy.Subscriber("/ptu/vel",Joy,control_callback)
	rate = rospy.Rate(10)
	init_count = 0
	while not rospy.is_shutdown():
		if init_count < 15:
			vector = Vector3()
			vector.x = 0
			vector.y = 0
			pub.publish(vector)
			init_count=init_count+1
			print 'init'
		rate.sleep()

main()


