#!/usr/bin/env python

import dvrk
import rospy
import tf
import math
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo
from opencv_apps.msg import Line
from featurization.msg import FeaturePoints
from image_geometry import StereoCameraModel
import PyKDL

class motion_controller:

    def __init__(self, arm_name, camera_model, tf_listener):
        rospy.loginfo('Configuring motion controller for ' + arm_name)
        self.arm = dvrk.arm(arm_name)
        self.arm_name = arm_name
        self.arm_base = arm_name + "_base"
        self.camera_model = camera_model
        self.tf_listener = tf_listener

        self.position_threshold = 1e-5
        self.joint_threshold = 3 * math.pi / 180

    def move_arm(self, feature_points_left, feature_points_right):
        desired_pose, yaw_angle = self.calc_desired_pose(feature_points_left, feature_points_right)
        self.cartesian_goal(desired_pose)
        self.joint_goal(yaw_angle)

    def calc_desired_pose(self, feature_points_left, feature_points_right):
        left_x = feature_points_left.center.x
        left_y = feature_points_left.center.y
        right_x = feature_points_right.center.x

        # Calculate disparity between images
        disparity = abs(right_x - left_x)

        (x, y, z) = self.camera_model.projectPixelTo3d(
            (left_x, left_y), disparity)

        x = x - 0.0025  # center between both cameras

        rospy.loginfo("Feature position in camera frame:")
        rospy.loginfo(x)
        rospy.loginfo(y)
        rospy.loginfo(z)

        rospy.loginfo("Feature angle in camera frame: " + str(feature_points_left.yaw_angle.data * 180 / math.pi))

        # Create desired pose in camera frame
        feature = PoseStamped()
        feature.header.stamp = rospy.Time.now()
        feature.header.frame_id = "camera"
        feature.pose.position.x = x
        feature.pose.position.y = y
        feature.pose.position.z = z

        # Convert desired angle to quaternion
        quaternion = tf.transformations.quaternion_from_euler(
            feature_points_left.yaw_angle.data, 0.0, 0.0)
        feature.pose.orientation.w = quaternion[0]
        feature.pose.orientation.x = quaternion[1]
        feature.pose.orientation.y = quaternion[2]
        feature.pose.orientation.z = quaternion[3]

        current_zoom = np.linalg.norm(z)
        rospy.loginfo("Zoom Level:")
        rospy.loginfo(current_zoom)

        desired_pose, rel_pose = self.transform_feature_pose(feature)

        desired_dir = np.array([desired_pose.pose.position.x,
                                desired_pose.pose.position.y, desired_pose.pose.position.z])
        desired_dir_unit = desired_dir / np.linalg.norm(desired_dir)

        zoom_offset = current_zoom * desired_dir_unit
        zoom_buffer = 2  # make sure no features are cut out
        zoom_offset = (
            zoom_offset / feature_points_left.zoom.data) * zoom_buffer

        desired_pose.pose.position.x -= zoom_offset[0]
        desired_pose.pose.position.y -= zoom_offset[1]
        desired_pose.pose.position.z -= zoom_offset[2]
        desired_pose.pose.orientation = feature.pose.orientation

        rospy.loginfo("Relative Desired Pose: ")
        rospy.loginfo(rel_pose)

        return desired_pose, feature_points_left.yaw_angle.data
    
    def transform_feature_pose(self, feature):
        try:
            time = feature.header.stamp
            self.tf_listener.waitForTransform(
                self.arm_base, "camera", time, rospy.Duration(3.0))
            desired_pose = self.tf_listener.transformPose(self.arm_base, feature)
            rel_pose = self.tf_listener.transformPose(self.arm_name, feature)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to calculate cartesian position")
            return
        
        return desired_pose, rel_pose

    def move_six_dof_arm(self, affine_tf):
        rospy.loginfo("AFFINE")
        rospy.loginfo(affine_tf)

    def cartesian_goal(self, goal):
        rospy.loginfo(rospy.get_caller_id() + '-> starting cartesian goal')
        self.prepare_cartesian()

        cart_goal = PyKDL.Frame()
        cart_goal.p = self.arm.get_desired_position().p
        cart_goal.M = self.arm.get_desired_position().M

        # Translation motion
        cart_goal.p[0] = goal.pose.position.x
        cart_goal.p[1] = goal.pose.position.y

        if goal.pose.position.z > -0.120:
            cart_goal.p[2] = -0.121
            rospy.loginfo("Defaulting to -0.121 z value")
        else:
            cart_goal.p[2] = goal.pose.position.z

        # # Rotation motion
        # cart_goal.M = PyKDL.Rotation.Quaternion(
        #     goal.pose.orientation.x,
        #     goal.pose.orientation.y,
        #     goal.pose.orientation.z, goal.pose.orientation.w)

        # r, p, yaw = cart_goal.M.GetRPY()
        # x, y, z, w = cart_goal.M.GetQuaternion()
        # rospy.loginfo(x)
        # rospy.loginfo("roll angle: " + str(r * 180 / 3.1415926))
        # rospy.loginfo("pitch: " + str (p * 180 / 3.1415926))
        # rospy.loginfo("Yaw angle: " + str(yaw * 180 / 3.1415926))
        # feature_points_right = rospy.wait_for_message(
        # "/featurization/right/feature_points", FeaturePoints, timeout=None)

        # self.joint
        
        if (self.check_cartesian_error(cart_goal) < self.position_threshold):
            rospy.loginfo('Position is within position threshold, no goal will be sent.')
        else:
            # Send goal
            self.arm.move(cart_goal.p)
            
            error = self.check_cartesian_error(cart_goal)
            rospy.loginfo('Inverse kinematic error in position: ' + str(error))
            rospy.loginfo('Cartesian goal complete')
    
    def check_cartesian_error(self, cart_goal):
        # check error on kinematics, compare to desired on arm.
        # to test tracking error we would compare to
        # current_position
        current_position = PyKDL.Frame()
        current_position.p = self.arm.get_current_position().p
        rospy.loginfo("CURRENT POSITION: ")
        rospy.loginfo(current_position.p)

        rospy.loginfo("GOAL POS")
        rospy.loginfo(cart_goal.p)
        errorX = cart_goal.p[0] - current_position.p[0]
        errorY = cart_goal.p[1] - current_position.p[1]
        errorZ = cart_goal.p[2] - current_position.p[2]
        error = math.sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ)
        
        rospy.loginfo("Erroy X = " + str(errorX))
        rospy.loginfo("Erroy Y = " + str(errorY))
        rospy.loginfo("Erroy Z = " + str(errorZ))

        rospy.loginfo("Error " + str(error))
        return error    
      

    def joint_goal(self, yaw_angle):
        orientation_goal = np.copy(self.arm.get_current_joint_position())

        rospy.loginfo("Yaw " + str(yaw_angle * 180 / 3.14159))

        orientation_goal[3] -= yaw_angle  # Set yaw joint

        if (self.check_joint_error(orientation_goal) < self.joint_threshold):
            rospy.loginfo('Orientation within joint threshold, no goal will be sent.')
        else:
            self.arm.move_joint(orientation_goal, interpolate=True)
            error = self.check_joint_error(orientation_goal)
            rospy.loginfo('Inverse kinematic error in position in yaw (deg): ' +
                str(error * 180 / math.pi))
            rospy.loginfo('Joint goal complete')

    def check_joint_error(self, orientation_goal):
        # check error on kinematics, compare to desired on arm.
        # to test tracking error we would compare to
        # current_position
        current_joint_position = np.copy(
            self.arm.get_current_joint_position())
        error = orientation_goal[3] - current_joint_position[3]
        return error
        

    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        current_joint_position = np.copy(
            self.arm.get_current_joint_position())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            if (current_joint_position[2] < 0.12):
                rospy.loginfo("Homing...")
                self.home()
    
    def home(self):
        rospy.loginfo('Homing...')
        self.arm.home()
        # get current joints just to set size
        goal = np.copy(self.arm.get_current_joint_position())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        self.arm.move_joint(goal, interpolate=True)