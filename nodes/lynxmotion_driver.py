#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_gimbal")
import rospy
from falkor_msgs.msg import *
import numpy as np

class LynxmotionDriver:
    def __init__(self):
        self.yaw_max = np.pi/4
        self.yaw_min = -np.pi/4
        self.pitch_max = np.pi/4
        self.pitch_min = -np.pi/4

        self.pwm_topic = rospy.get_param( "~pwm_topic", "pwm_out" )
        self.pwm_pub = rospy.Publisher( self.pwm_topic, Pwm )
        self.pwm_min = 1000
        self.pwm_max = 2000

        self.gimbal_topic = rospy.get_param( "~gimbal_topic", "gimbal_cmd" )
        self.gimbal_sub = rospy.Subscriber( self.gimbal_topic, Gimbal, self.gimbal_cb )

    def to_pwm( self, radians, min_rad, max_rad ):
        return ( radians - min_rad ) / ( max_rad - min_rad ) * ( self.pwm_max - self.pwm_min ) + self.pwm_min

    def gimbal_cb( self, data ):
        pitch_pwm = self.to_pwm( data.pitch, self.pitch_min, self.pitch_max )
        yaw_pwm = self.to_pwm( -data.yaw, self.yaw_min, self.yaw_max )
        pwm_msg = Pwm( [ int(pitch_pwm), int(yaw_pwm) ] )
        self.pwm_pub.publish( pwm_msg )

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('lynxmotion_driver')

    driver = LynxmotionDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

