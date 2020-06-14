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

class feature_processor:

    def __init__(self, feature_files, desired_viewpoint, six_dof=False):
        # Set hsv lower and upper limits
        self.StoreHSVRanges(feature_files)
        self.six_dof = six_dof
        # Declare adjustment values to match cv2 hsv value storage
        self.hsv_adjustment = np.array(
            [1.0 / 2.0, 255.0 / 100.0, 255.0 / 100.0])

        self.min_contour_area = 10

        self.StoreViewpoint(desired_viewpoint)

        # Declare a cv to ros bridge
        self.bridge = CvBridge()

    def StoreHSVRanges(self, feature_files):
        hsv_ranges = np.empty((1,  2, 3))
        rospy.loginfo("Number of feature files:")
        rospy.loginfo(len(feature_files))
        for feature in feature_files:
            feature_range = np.genfromtxt(feature, delimiter=',')
            range_reshaped = np.reshape(feature_range,  (1, 2, 3))
            hsv_ranges = np.append(hsv_ranges, range_reshaped, axis=0)

        # Delete empty row
        hsv_ranges = np.delete(hsv_ranges,  0, axis=0)
        rospy.loginfo("Feature ranges: ")
        rospy.loginfo(hsv_ranges)

        self.hsv_ranges = hsv_ranges
        self.n_features = hsv_ranges.shape[0]

    def StoreViewpoint(self, desired_viewpoint):
        # Get frame from desired viewpoint image
        frame = cv2.imread(desired_viewpoint, cv2.IMREAD_COLOR)

        # Convert to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        centroids = np.empty((1, 2))
        for f in range(self.n_features):
            # The lower and upper ranges of HSV recognition for this feature
            lower_range = self.hsv_ranges[f, 0, :]
            upper_range = self.hsv_ranges[f, 1, :]

            contours = self.FindContours(hsv, lower_range, upper_range)
            rospy.loginfo(f)
            # Iterate through all contours
            for c in contours:

                # Skip contours with areas that are too small
                if (cv2.contourArea(c) < self.min_contour_area):
                    continue
                
                if (self.six_dof):
                    self.viewpoint_hull = self.HullWithNCorners(c, frame)
                    return
                # # Find the x and y coordinates of the centroid of the object.
                center, _ = cv2.minEnclosingCircle(c)

                cx = int(center[0])
                cy = int(center[1])

                # Store the centroids
                centroids = np.append(centroids, [[cx, cy]], axis=0)

        # Delete first empty rows
        self.viewpoint_points = np.delete(centroids,  0, axis=0)
        self.viewpoint_size = self.GetShapeSize(self.viewpoint_points)

    def GetShapeSize(self, points):
        # Store area
        if (points.shape[0] == 0):
            rospy.logerr("No feature points to find size of!")
            size = 0
        if(points.shape[0] == 1):
            # Size is zero
            size = 0
        if (points.shape[0] == 2):
            # Store length line instead of area
            size = np.linalg.norm(
                points[0, :] - points[1, :])
        else:
            size = cv2.contourArea(np.float32(points))
        rospy.loginfo("Size: "+ str(size))
        return size

    def PrepareImage(self, ros_image):
        # try catch block to capture exception handling
        try:
            # Convert ROS message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                ros_image, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.loginfo(e)

        # Draw a circle outline at the centre of the frame
        height, width = frame.shape[0:2]
        cv2.circle(frame, (int(width / 2), int(height / 2)),
                   radius=2, color=(0, 0, 0), thickness=1)

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

    def GenerateFeaturePoints(self, frame, points, point_type):
        feature_points = FeaturePoints()
        feature_points.type.data = point_type

        # Calculate center of points
        [cpoint_x, cpoint_y] = np.int32(np.average(points, axis=0))

        feature_points.center = Point2D(cpoint_x, cpoint_y)

        # Creates a circle outline at the center of points
        cv2.circle(frame, (cpoint_x, cpoint_y), radius=2,
                   color=(248, 123, 191), thickness=1)

        feature_points.points = []
        # Fill array of points with points
        for i in range(points.shape[0]):
            center_point = Point2D(points[i, 0], points[i, 1])
            feature_points.points.append(center_point)
        
        rospy.loginfo(feature_points.points)        
        poly_points = np.int32([points])
        # Draw lines between the points
        cv2.polylines(frame, poly_points, color=(
            248, 123, 191), thickness=1, isClosed=True)
        
        # Set zoom level
        rospy.loginfo(np.float32(points))
        size = self.GetShapeSize(points)

        if size == 0.0:
            # Features are out of frame, don't do anything
            feature_points.zoom.data = 1.0
            feature_points.yaw_angle.data = 0.0
            return feature_points

        feature_points.zoom.data = self.viewpoint_size / size
        # rospy.loginfo("Zoom: " + str(feature_points.zoom))

        # Set angle
        tform = sktf.estimate_transform('similarity', np.float32(points), np.float32(self.viewpoint_points))
        rospy.loginfo(tform.rotation * 180 / math.pi)
        
        if (tform.rotation > math.pi / 2):
            angle = -1 * (math.pi - tform.rotation)
        else:
            angle = tform.rotation

        # angle = tform.rotation
        # rospy.loginfo("Yaw Angle: " + str(angle * 180 / math.pi))
        feature_points.yaw_angle.data = angle

        # return points
        return feature_points

    def ShowFrame(self, frame, name):
        # Shows the centroid point in the live feed
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, frame)
        cv2.waitKey(delay=30)

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

                # Draw the contour lines
                cv2.drawContours(frame, [c], -1, (0, 0, 0), 1)

                # # Find the x and y coordinates of the centroid of the object.
                center, _ = cv2.minEnclosingCircle(c)

                cx = int(center[0])
                cy = int(center[1])

                # Store the centroids
                centroids = np.append(centroids, [[cx, cy]], axis=0)

                # Creates a circle at the centroid point
                cv2.circle(frame, (cx, cy), 3, (0, 0, 0), -1)
        
       

        # Delete first empty rows
        centroids = np.delete(centroids,  0, axis=0)

        if (centroids.size < 2):
            rospy.logerr(
                'No centroids were found! Nothing will be published.')
        else:
            feature_points = self.GenerateFeaturePoints(frame, centroids,"centroids")

        self.ShowFrame(frame, name)

        return feature_points

    def HullWithNCorners(self, contour, frame, closed = True, n_corners=8):
        """
        Finds four corners from a list of points on the goal
        epsilon - the minimum side length of the polygon generated by the corners

        Parameters:
            :param: `contour` - a numpy array of points (opencv contour) of the
                                points to get corners from
            :param: `n_corners` - the number of corners to find
        """
        coefficient = .05
        while True:
            # rospy.loginfo(contour)
            epsilon = coefficient * cv2.arcLength(contour, closed)
            # epsilon =
            # rospy.loginfo("epsilon:", epsilon)
            poly_approx = cv2.approxPolyDP(contour, epsilon, True)
            hull = cv2.convexHull(poly_approx)
            # rospy.loginfo(hull.shape)
            if len(hull) == n_corners:
                rospy.loginfo("hull")
                rospy.loginfo(hull)
                rospy.loginfo(hull.shape)
                hull = hull.reshape((n_corners, 2))
                rospy.loginfo(hull.shape)
                rospy.loginfo(hull)
                # Creates a circle at each  hull point
                for h in range(n_corners):
                    cv2.circle(
                        frame, (hull[h, 0], hull[h, 1]), 3, (0, 255, 0), -1)

                return hull
            else:
                if len(hull) > n_corners:
                    print(len(hull))
                    coefficient += .01
                else:
                    print(len(hull))
                    coefficient -= .01

    def FittedContourPoints(self, ros_image, name):

        frame, hsv = self.PrepareImage(ros_image)

        # The lower and upper ranges of HSV recognition for this feature
        lower_range = self.hsv_ranges[0, 0, :]
        upper_range = self.hsv_ranges[0, 1, :]

        # Find contours in hsv image
        contours = self.FindContours(hsv, lower_range, upper_range)

        for c in contours:

            # Skip contours with areas that are too small
            if (cv2.contourArea(c) < self.min_contour_area):
                continue

            # Draw the contour lines
            cv2.drawContours(frame, [c], -1, (0, 0, 0), 1)
            # rospy.loginfo("contour")
            # rospy.loginfo(c)

            hull = self.HullWithNCorners(c, frame)

            break

        affine_tf = cv2.estimateRigidTransform(hull, self.viewpoint_hull, fullAffine=True)
        print(affine_tf)
        self.ShowFrame(frame, name)
        
        return affine_tf
        # self.PublishPoints(frame, hull, "fitted_contour")
