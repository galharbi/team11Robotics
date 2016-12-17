#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2


# Instantiate CvBridge
bridge = CvBridge()

increment = 0

def image_callback(msg):
    print("Received an image!")
    global increment
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('high_res_marker' + str(increment) + '.jpg', cv2_img)
        increment += 1
        rospy.sleep(5)
    except CvBridgeError, e:
        print(e)
        # Save your OpenCV2 image as a jpeg 
        
    return None
        


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/cameras/left_hand_camera/image"
    # Set up your subscriber and define its callback
    s = rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    raw_input('Press Enter: ')
    s.unregister()
     

if __name__ == '__main__':
    while True:
    	r = raw_input('Press Enter: ')
        if r != '':
 	    break
    	main()


