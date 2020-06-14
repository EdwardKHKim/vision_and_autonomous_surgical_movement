#!/usr/bin/env python

import test_ecm
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL

class movement:

  # configuration
  def configure(self, robot_name):
    print(rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name)
    self.psm = dvrk.psm(robot_name)

    # homing example
  def home(self):
    print(rospy.get_caller_id(), ' -> starting home')
    self.psm.home()
    # get current joints just to set size
    goal = numpy.copy(self.psm.get_current_joint_position())
    # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
    goal.fill(0)
    if ((self.psm.name() == 'PSM1') or (self.psm.name() == 'PSM2') or (self.psm.name() == 'PSM3') or (self.psm.name() == 'ECM')):
        goal[2] = 0.12
    self.psm.move_joint(goal, interpolate = True)

 # direct cartesian control example
  def cartesian_goal(self, value_x, value_y, value_z):
    print(rospy.get_caller_id(), ' -> starting cartesian goal')

    current_position = self.psm.get_current_position().p
    new_value_x = value_x - current_position[0]
    new_value_y = value_y - current_position[1]
    new_value_z = value_z - current_position[2]
    self.psm.dmove(PyKDL.Vector(new_value_x, new_value_y, new_value_z))
    print(rospy.get_caller_id(), ' <- cartesian goal complete')

    self.psm.open_jaw()
    self.psm.close_jaw()

if __name__ == '__main__':
  if (len(sys.argv) != 2):
    print(sys.argv[0], ' requires one argument, i.e. name of dVRK arm')
  else:
    application = movement()
    application.configure(sys.argv[1])
    application.home()
    application.cartesian_goal( 0.13618, 0.031615, -0.116062)
    application.cartesian_goal( 0.0989808, -0.0482103, -0.0843597)
