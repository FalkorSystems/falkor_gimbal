#!/usr/bin/env python
import cv2
import numpy as np

capture = cv2.VideoCapture( "/devices/pci0000:00/0000:00:11.0/0000:02:03.0/usb1/1-1/1-1:1.0/input/input7" )
print capture.isOpened()

while True:
    retval, frame = capture.read()
    import pdb; pdb.set_trace()
    cv2.imshow( 'Raw', frame )

