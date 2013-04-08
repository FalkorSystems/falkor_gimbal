#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_gimbal")
import rospy
from falkor_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from dynamixel_msgs.msg import *
from std_msgs.msg import *
import numpy as np

class Follow:
    def __init__(self):

        self.pan_state = rospy.Subscriber( "pan_controller/state", JointState, self.pan_state_cb )
        self.tilt_state = rospy.Subscriber( "tilt_controller/state", JointState, self.tilt_state_cb )

        self.pan_pub = rospy.Publisher( "pan_controller/command", Float64 )
        self.tilt_pub = rospy.Publisher( "tilt_controller/command", Float64 )

        rospy.sleep( 2 )
        self.pan_pub.publish( Float64( 0.0 ) )
        self.tilt_pub.publish( Float64( 0.0 ) )
        rospy.sleep( 2 )

        self.camera_info = CameraInfo()
        self.camera_info_topic = rospy.get_param( "~camera_info_topic", "camera/camera_info" )
        self.camera_info_sub = rospy.Subscriber( self.camera_info_topic, CameraInfo, self.camera_info_cb )

        self.found_point = Point()

        # Run at Hz
        self.gimbal_cmd_limit = np.pi

        self.found_point_topic = rospy.get_param( "~found_point_topic", "found_point" )
        self.found_point_sub = rospy.Subscriber( self.found_point_topic, Point, self.found_point_cb )

    def pan_state_cb( self, data ):
        self.pan_state = data

    def tilt_state_cb( self, data ):
        self.tilt_state = data

    def camera_info_cb( self, data ):
        self.camera_info = data

    def limit_move( self, angle ):
        return min( max( angle, -self.gimbal_move_limit ), self.gimbal_move_limit )

    def limit_cmd( self, angle ):
        return min( max( angle, -self.gimbal_cmd_limit ), self.gimbal_cmd_limit )

    def found_point_cb( self, data ):
        self.found_point = data

        if self.found_point.x == -1:
            return

        f_x = self.camera_info.P[0]
        c_x = self.camera_info.P[2]
        f_y = self.camera_info.P[5]
        c_y = self.camera_info.P[6]
        height = self.camera_info.height
        width = self.camera_info.width

        pan = -np.arctan2( self.found_point.x - width/2, f_x )
        tilt = np.arctan2( self.found_point.y - height/2, f_y )

        pan_goal = self.pan_state.current_pos + pan
        tilt_goal = self.tilt_state.current_pos + tilt

        self.pan_pub.publish( Float64( pan_goal ) )
        self.tilt_pub.publish( Float64( tilt_goal ) )

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('follow')

    follower = Follow()
    try:
        follower.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()


