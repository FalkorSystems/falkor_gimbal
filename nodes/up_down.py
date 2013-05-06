#!/usr/bin/env python
import roslib; roslib.load_manifest("falkor_gimbal")
import rospy
import random

from dynamixel_msgs.msg import *
from std_msgs.msg import *
import numpy as np

rospy.init_node( 'up_down' )
tilt_pub = rospy.Publisher( "tilt_controller/command", Float64 )

current_tilt = 0.0
max_tilt = np.pi/4
tilt_step = np.pi/32
center_pause = 5
step_pause = 0.3
direction = 1

while True:
    if current_tilt == 0.0:
        rospy.sleep( center_pause )
    else:
        rospy.sleep( step_pause )
    
    current_tilt += direction * tilt_step
    if abs( current_tilt ) >= max_tilt:
        direction = -direction
    tilt_pub.publish( Float64( current_tilt ) )
