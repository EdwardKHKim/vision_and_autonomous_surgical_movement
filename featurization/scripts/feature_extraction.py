#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from featurization.msg import FeaturePoints
from opencv_apps.msg import Point2D

import cv2
import numpy as np
import imutils
import sys
import csv
import math
from skimage import transform as sktf

class FeatureExtraction:

    def __init__(self, hsv_ranges, desired_viewpoint):
        # Set hsv lower and upper limits
        self.hsv_ranges = hsv_ranges
        self.n_features = hsv_ranges.shape[0]
        # Declare adjustment values to match cv2 hsv value storage
        self.hsv_adjustment = np.array(
            [1.0 / 2.0, 255.0 / 100.0, 255.0 / 100.0])

        self.pub = rospy.Publisher(
            '/featurization/left/feature_points', FeaturePoints, queue_size=100)

        self.min_contour_area = 10

        # Declare a cv to ros bridge
        self.bridge = CvBridge()


    def PrepareImage(self, ros_image):
        # try catch block to capture exception handling
        try:
            # Convert ROS message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                ros_image, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)

        # Covert OpenCV image into gray scale
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        return frame, hsv

    def FindContours(self, hsv, lower_range, upper_range):
        # Masks the input frame using the HSV upper and lower range
        mask = cv2.inRange(hsv, lower_range * self.hsv_adjustment,
                           upper_range * self.hsv_adjustment)

        # Create contours of the segmented diagram from the HSV
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        return contours

    def PublishPoints(self, frame, points, point_type):
        feature_points = FeaturePoints()
        feature_points.type.data = point_type

        # Calculate center of points
        [x, y] = np.int32(np.average(points, axis=0))

        feature_points.center = Point2D(x, y)

        feature_points.points = []
        # Fill array of points with points
        for i in range(points.shape[0]):
            center_point = Point2D(points[i, 0], points[i, 1])
            feature_points.points.append(center_point)
        
        print(feature_points.points)        

        # Publish points
        self.pub.publish(feature_points)

    def Centroids(self, ros_image, name):
        # Prepare image for processing
        frame, hsv = self.PrepareImage(ros_image)

        centroids = np.empty((1, 2))
        for f in range(self.n_features):
            # The lower and upper ranges of HSV recognition for this feature
            lower_range = self.hsv_ranges[f, 0, :]
            upper_range = self.hsv_ranges[f, 1, :]

            contours = self.FindContours(hsv, lower_range, upper_range)

            # Iterate through all contours
            for c in contours:

                # Skip contours with areas that are too small
                if (cv2.contourArea(c) < self.min_contour_area):
                    continue

                # # Find the x and y coordinates of the centroid of the object.
		moments = cv2.moments(c)

                cx = int(moments["m10"]/moments["m00"])
                cy = int(moments["m01"]/moments["m00"])

		# Draw the contour lines
                cv2.drawContours(frame, [c], 0, (0, 0, 0), 2)

                # Store the centroids
                centroids = np.append(centroids, [[cx, cy]], axis=0)

                # Creates a circle at the centroid point
                cv2.circle(frame, (cx, cy), 3, (0, 0, 0), -1)

        # Delete first empty rows
        centroids = np.delete(centroids,  0, axis=0)

	if (centroids.size < 2):
		rospy.logerr('No centroids were found')
	else:
		self.PublishPoints(frame, centroids, "centroids")

        # Shows the centroid point in the live feed
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, frame)
        cv2.waitKey(delay=30)

def CSVToHSVRanges(features):
    hsv_ranges = np.empty((1,  2, 3))
    print(len(features))
    for feature in features:
        feature_range = np.genfromtxt(feature, delimiter=',')
        range_reshaped = np.reshape(feature_range,  (1, 2, 3))
        hsv_ranges = np.append(hsv_ranges, range_reshaped, axis=0)

    # Delete empty row
    hsv_ranges = np.delete(hsv_ranges,  0, axis=0)
    return hsv_ranges


if __name__ == '__main__':
    rospy.init_node('featurization', anonymous=True)
    ns = rospy.get_name()

    # Get default feature path
    r = rospkg.RosPack()
    path = r.get_path('featurization')
    # default_path = [path + '/config/tools.csv', path + '/config/red_ball.csv']
    default_path = [path + '/config/features/tools.csv',
                    path + '/config/features/red_ball.csv']
    # default_path = [path + '/config/features/tools.csv']
    desired_viewpoint = path + '/config/viewpoints/tools_red_ball_vp.jpg'
    feature_files = rospy.get_param('feature_files', default_path)

    hsv_ranges = CSVToHSVRanges(feature_files)

    featurization = FeatureExtraction(hsv_ranges, desired_viewpoint)

    rospy.Subscriber("/stereo/left/image_flipped", Image,
                     featurization.Centroids, callback_args=ns)

    # Makes sure that the live feed does not close instantly
    cv2.waitKey(0)

    # After live feed is closed code block ends
    cv2.destroyAllWindows()

    rospy.spin()
