#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_gimbal")
import rospy
from falkor_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import numpy as np
import pid

class Follow:
    def __init__(self):
        self.gimbal_cmd = Gimbal( 0, 0, 0 )
        self.gimbal_topic = rospy.get_param( "~gimbal_topic", "gimbal_cmd" )
        self.gimbal_pub = rospy.Publisher( self.gimbal_topic, Gimbal )

        rospy.sleep(2)
        self.gimbal_pub.publish( self.gimbal_cmd )


        self.found_point_topic = rospy.get_param( "~found_point_topic", "found_point" )
        self.found_point_sub = rospy.Subscriber( self.found_point_topic, Point, self.found_point_cb )

        self.camera_info = CameraInfo()
        self.camera_info_topic = rospy.get_param( "~camera_info_topic", "camera/camera_info" )
        self.camera_info_sub = rospy.Subscriber( self.camera_info_topic, CameraInfo, self.camera_info_cb )

        self.found_point = Point()

        # Run at Hz
        self.rate = rospy.Rate( 20 )

        self.gimbal_move_limit = np.pi/40
        self.yaw_pid = pid.Pid( 0.5, 0.0, 0.0, 0.0, self.gimbal_move_limit )
        self.pitch_pid = pid.Pid( 0.5, 0.0, 0.0, 0.0, self.gimbal_move_limit )

        self.gimbal_cmd_limit = np.pi/4
        self.time = rospy.Time.now()

    def camera_info_cb( self, data ):
        self.camera_info = data

    def limit_move( self, angle ):
        return min( max( angle, -self.gimbal_move_limit ), self.gimbal_move_limit )

    def limit_cmd( self, angle ):
        return min( max( angle, -self.gimbal_cmd_limit ), self.gimbal_cmd_limit )

    def found_point_cb( self, data ):
        self.found_point = data

    def run_once( self ):
        if self.found_point.x == -1:
            return
        f_x = self.camera_info.P[0]
        c_x = self.camera_info.P[2]
        f_y = self.camera_info.P[5]
        c_y = self.camera_info.P[6]
        height = self.camera_info.height
        width = self.camera_info.width

        yaw = -np.arctan2( self.found_point.x - width/2, f_x )
        pitch = np.arctan2( self.found_point.y - height/2, f_y )

        now = rospy.Time.now()

        dt = now - self.time
        self.time = now
        pitch_controlled = self.pitch_pid.update( pitch, 0, 0, dt.to_sec() )
        yaw_controlled = self.yaw_pid.update( yaw, 0, 0, dt.to_sec() )

        self.gimbal_cmd.pitch += pitch_controlled
        self.gimbal_cmd.yaw += yaw_controlled

        data = [ i / np.pi * 180 for i in ( pitch, yaw, pitch_controlled, yaw_controlled ) ]
        print "(%3d, %3d) -> (%3d, %3d)" % tuple(data)

        self.gimbal_cmd.pitch = self.limit_cmd( self.gimbal_cmd.pitch )
        self.gimbal_cmd.yaw = self.limit_cmd( self.gimbal_cmd.yaw )

        self.gimbal_pub.publish( self.gimbal_cmd )

    def run(self):
        while not rospy.is_shutdown():
            self.run_once()
            self.rate.sleep()

def main():
    rospy.init_node('follow')

    follower = Follow()
    try:
        follower.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()


