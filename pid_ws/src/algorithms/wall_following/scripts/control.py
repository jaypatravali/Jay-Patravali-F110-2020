#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math

KP = -0.12
KD = 0.00
ANGLE_LEVEL_1 = 10.0
SPEED_LEVEL_1 = 1.5
ANGLE_LEVEL_2 = 20.0
SPEED_LEVEL_2 = 1.0
SPEED_LEVEL_3 = 0.5

past_error = 0.0
past_timeStamp = 0.0
flag = False
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):

    curr_error = msg.data
    curr_timeStamp = rospy.Time.now().to_sec()

    global past_error
    global past_timeStamp
    global flag
    delta_time = abs(curr_timeStamp - past_timeStamp)
    delta_error = curr_error - past_error

    past_error = curr_error
    past_timeStamp = curr_timeStamp

    if flag:
        pidoutput = KP*curr_error + KD*(delta_error/delta_time)
    else:
        flag = True
        pidoutput = KP*curr_error


    angle = math.radians(pidoutput)    #convert the angle to radians if not already in radians
    angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    if abs(angle) < ANGLE_LEVEL_1:
        vel =  SPEED_LEVEL_1
    elif  ANGLE_LEVEL_1<= abs(angle) and abs(angle) < ANGLE_LEVEL_2:
    # elif abs(angle )< ANGLE_LEVEL_2:        
        vel =  SPEED_LEVEL_2
    else:
        vel =  SPEED_LEVEL_3
    # print (vel, angle)
    msg = drive_param()
    msg.velocity = vel  # TODO: implement PID for velocity
    msg.angle = angle    # TODO: implement PID for steering angle
    pub.publish(msg)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("pid_error", Float64, control_callback)
    rospy.spin()
