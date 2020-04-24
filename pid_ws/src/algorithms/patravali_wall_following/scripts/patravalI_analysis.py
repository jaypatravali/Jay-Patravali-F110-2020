#!/usr/bin/env python

import rospy
from patravali_wall_finding.msg import error_analysis
from std_msgs.msg import Float64
import numpy as np
import math

pub = rospy.Publisher('wall_following_analysis', drive_param, queue_size=1)

MAX_ERROR = -100.0
SUM_ERROR = 0.0
COUNTER = 1

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def error_callback(msg):
	global COUNTER
	global MAX_ERROR
	global AVG_ERROR

	if msg.data>MAX_ERROR:
		MAX_ERROR = msg.data

	SUM_ERROR += msg.data

	error_msg = error_analysis()
	error_msg.avg_abs_error = (SUM_ERROR/COUNTER)
	error_msg.max_error = MAX_ERROR

	COUNTER +=1
	rospy.loginfo("Running Avg Error %f and Max Error %f")


# Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('patravali_analyis', anonymous=True)
    rospy.Subscriber("pid_error", Float64, error_callback)
    rospy.spin()

