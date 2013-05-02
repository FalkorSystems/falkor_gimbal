#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_gimbal")
import rospy
from falkor_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
import cv, cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Track:
    def __init__(self):
        self.found_point_pub = rospy.Publisher( "found_point", PointStamped )

        self.params = cv2.SimpleBlobDetector_Params()
        # params.minDistBetweenBlobs = 10
        # params.filterByInertia = False

        # params.filterByConvexity = False
        # params.minConvexity = 0.5
        # params.maxConvexity = 1.0

        self.params.filterByColor = False
        self.params.blobColor = 127

        self.params.filterByCircularity = False
        self.params.minCircularity = 0.5
        self.params.maxCircularity = 2.0

        self.params.filterByArea = True
        self.params.minArea = 2000
        self.params.maxArea = 7000
        
        self.detector = cv2.SimpleBlobDetector( self.params ) 
        self.bridge = CvBridge()

        self.minHue = 35
        self.maxHue = 90

        # cv2.namedWindow( "Controls" )
        # cv2.createTrackbar( "minArea", "Controls", int(self.params.minArea), 10000, self.update_minArea )
        # cv2.createTrackbar( "maxArea", "Controls", int(self.params.maxArea), 10000, self.update_maxArea )

        # cv2.createTrackbar( "minHue", "Controls", self.minHue, 180, self.update_minHue )
        # cv2.createTrackbar( "maxHue", "Controls", self.maxHue, 180, self.update_maxHue )

        # cv2.createTrackbar( "minCircularity", "Controls", int(self.params.minCircularity*100), 200, self.update_minCircularity )
        # cv2.createTrackbar( "maxCircularity", "Controls", int(self.params.maxCircularity*100), 200, self.update_maxCircularity )

        # cv2.createTrackbar( "blobColor", "Controls", self.params.blobColor, 255, self.update_blobColor )
        # cv2.createTrackbar( "maxCircularity", "Controls", int(self.params.maxCircularity*100), 200, self.update_maxCircularity )

        self.image_pub = rospy.Publisher( "camera/image_track_color", Image )
        self.image_sub = rospy.Subscriber( "camera/image_rect_color", Image, self.image_cb )

    def update_blobColor( self, value ):
        if value < 10:
            self.params.blobColor = 0
            self.params.filterByColor = True
        elif value > 250:
            self.params.blobColor = 255
            self.params.filterByColor = True
        else:
            self.params.filterByColor = False
            
        self.update_params()
    
    def update_minHue( self, value ):
        self.minHue = value

    def update_maxHue( self, value ):
        self.maxHue = value

    def update_minArea( self, value ):
        self.params.minArea = value
        self.update_params()

    def update_maxArea( self, value ):
        self.params.maxArea = value
        self.update_params()

    def update_minCircularity( self, value ):
        self.params.minCircularity = value/100.0
        self.update_params()

    def update_maxCircularity( self, value ):
        self.params.maxCircularity = value/100.0
        self.update_params()

    def update_params( self ):
        self.detector = cv2.SimpleBlobDetector( self.params )

    def track(self, image):
        scale = 10
        min_radius = 10
        image = cv2.resize( image, ( int(image.shape[1]/scale), int(image.shape[0]/scale )) )
        vis_image = image
        # convert to HSV
        image = cv2.cvtColor( image, cv2.cv.CV_BGR2HSV )
        image = cv2.inRange( image, np.array( ( self.minHue, 40, 00 ) ), np.array( ( self.maxHue, 255, 255 ) ) )
        image = cv2.morphologyEx( image, cv2.MORPH_OPEN, cv2.getStructuringElement( cv2.MORPH_RECT, ( 5, 5 ) ) )

        contours, hierarchy = cv2.findContours( image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE )
        circles = cv2.HoughCircles( image, cv2.cv.CV_HOUGH_GRADIENT, 1, 5 )

	contour_lengths = np.array( [ cv2.arcLength( i, True ) for i in contours ] )
        hulls = np.array( [ cv2.convexHull( i ) for i in contours ] )
#        import pdb; pdb.set_trace()
#        convexities = np.array( [ cv2.contourArea( i ) / cv2.contourArea( j ) for (i,j) in (contours,hulls) ] )

        if len( contour_lengths ) > 0:
            longest_contour = np.argmax( contour_lengths )
            cv2.drawContours( vis_image, contours, -1, ( 0, 255, 255 ) )
            cv2.drawContours( vis_image, contours, longest_contour, ( 0, 0, 255 ), 5 )
        
            center, radius = cv2.minEnclosingCircle(contours[longest_contour])
            if radius > min_radius/scale:
                center_col = 255
            else:
                center_col = 0    
            cv2.circle( vis_image, ( int(center[0]), int(center[1]) ), int(radius), (255, center_col, 255), -1 )
        
        if circles != None:
            for i in circles:
                cv2.circles( vis_image, ( int(i[0]), int(i[1]) ), int(i[3]), (0,255,0), -1 )

        image_msg = self.bridge.cv_to_imgmsg( cv2.cv.fromarray( vis_image ), encoding="passthrough" )
        self.image_pub.publish( image_msg )

        if len(contour_lengths) > 0 and radius > min_radius/scale:
            return Point( center[0] * scale, center[1] * scale, 0 )
        else:
            return Point( -1, -1, -1 )

    def image_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "rgb8" )
        except:
            print e

        numpy_image = np.asarray(cv_image)

        point = self.track( numpy_image )
        header = Header()
        header.stamp = data.header.stamp
        point_stamped = PointStamped( header, point )
        self.found_point_pub.publish( point_stamped )

    def run(self):
        while not rospy.is_shutdown():
            cv2.waitKey(1)

def main():
    rospy.init_node('track')

    tracker = Track()
    try:
        tracker.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()




        
