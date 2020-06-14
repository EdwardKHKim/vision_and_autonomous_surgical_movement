#!/usr/bin/env python
import rospy

import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import math
import cv2

# This node flips images from the simulation

def Flip(image, (axis, pub)):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')
    cv_image_flipped = cv2.flip(cv_image, axis)
    image_flipped = bridge.cv2_to_imgmsg(cv_image_flipped, encoding='rgb8')
    pub.publish(image_flipped)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('image_flipper', anonymous=True)

    # Declare publishers for stereo flipped image
    left_image_flipped = rospy.Publisher(
        '/stereo/left/image_flipped', Image, queue_size=1000)
    right_image_flipped = rospy.Publisher(
        '/stereo/right/image_flipped', Image, queue_size=1000)
    
    # Image topic subscribers
    # Left image is flipped on x axis
    rospy.Subscriber("/stereo/left/image_raw", Image,
                    Flip,  callback_args=(0, left_image_flipped))

    # Right image is flipped on y axis
    rospy.Subscriber("/stereo/right/image_raw", Image,
                    Flip,  callback_args=(0, right_image_flipped))
    rospy.spin()
