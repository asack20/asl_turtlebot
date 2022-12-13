#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

cmd_vel_nav = Twist()
cmd_vel_key = Twist()

rospy.init_node("turtlebot_multiplexer", anonymous=True)
rate = rospy.Rate(10)

def cmd_vel_nav_callback(data):
	cmd_vel_nav.angular = data.angular
	cmd_vel_nav.linear = data.linear
	
def cmd_vel_key_callback(data):
	cmd_vel_key.angular = data.angular
	cmd_vel_key.linear = data.linear

rospy.Subscriber("/navigator/cmd_vel", Twist, cmd_vel_nav_callback)
rospy.Subscriber("/teleop_key/cmd_vel", Twist, cmd_vel_key_callback)

cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

while not rospy.is_shutdown():
	control_mode = rospy.get_param("/multiplexer/control_mode")
	if control_mode == 'STOP':
		cmd_stop = Twist()
		# print('Supervisor cmd:', cmd_stop)
		cmd_vel_pub.publish(cmd_stop) 
	elif control_mode == 'NAV':
		cmd_vel_pub.publish(cmd_vel_nav)
		#print(control_mode)
	elif control_mode == 'KEY':
		cmd_vel_pub.publish(cmd_vel_key)
		#print(control_mode)
	else:
		print('No control mode set')
		
	rate.sleep()
