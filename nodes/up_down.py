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


while True:
    tilt_pub.publish( Float64( current_tilt ) )
    random_wait = random.gauss( 4, 2 )
    rospy.sleep(random_wait)

    tilt_change = random.gauss( 0, np.pi/2 )
    if current_tilt == -np.pi/4:
        tilt_change = abs( tilt_change )
    if current_tilt == np.pi/4:
        tilt_change = - abs( tilt_change )

    current_tilt += tilt_change
    current_tilt = min( max( current_tilt, -np.pi/4 ), np.pi/4 )

