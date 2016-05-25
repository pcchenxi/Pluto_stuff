#!/usr/bin/env python

'''
This program is to be ran alongside atrv atrvnode and joy joy_node.
The program reads the first and second axes on a plugged-in joystick, then prints these values and sends /atrv/cmd_vel Twist commands to control the atrv.
'''

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


velocity_global = 0.0
rotation_global = 0.0

high_speed_guard = True
stop_control = 1

def joystick_callback(joy_msg):
    global high_speed_guard,stop_control

    '''velocity and rotation of body'''
    global velocity_global,rotation_global,a_button_pressed,sonar_power_on
    vel = joy_msg.axes[1]
    rot = joy_msg.axes[0]

    #print joy_msg.axes[2],joy_msg.axes[3],joy_msg.axes[4],joy_msg.axes[5]

    vel_multiplier = 1-joy_msg.axes[2]+2-joy_msg.axes[5]*2 #The "fire" buttons

    if vel_multiplier<1.1:
        high_speed_guard = False

    #Deadzone
    if abs(vel)<0.13:
        vel = 0.0
    if abs(rot)<0.13:
        rot = 0.0

    if not high_speed_guard:
        velocity_global = vel*0.1 + vel*vel_multiplier
        rotation_global = rot #low rotational speed, otherwise the robot will stop to try to rotate even when tilting the controller axis slightly to the left or right.
    if joy_msg.buttons[4]:
	stop_control = -stop_control



def joystick_controller():
    print "use left pad to go forward"
    print "use left and right triggers to increase speed"
    print "press and release triggers to start"
    global stop_control

    velbound = 7 #bound on the forward/backward velocity
    rotbound = 1 #bound on the rotation. Good to have this much lower.
    pub = rospy.Publisher('/atrv/cmd_vel', Twist, queue_size=10)
    rospy.init_node('joystick_atrv_controller', anonymous=True)
    rospy.Subscriber("/joy", Joy, joystick_callback)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
	if stop_control == -1:
		continue

        twist = Twist()

        vel = velocity_global
        rot = rotation_global
        vel = min(velbound,vel)
        vel = max(-velbound,vel)
        rot = min(rotbound,rot)
        rot = max(-rotbound,rot)
        
        twist.linear.x = vel
        twist.angular.z = rot
        #print(twist.linear.x,twist.angular.z)

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    joystick_controller()
