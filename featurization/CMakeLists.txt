cmake_minimum_required(VERSION 2.8.3)
project(featurization)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED 
          roslaunch
          rospy
          opencv_apps
          message_generation
          )


add_message_files(
  FILES
  FeaturePoints.msg
)



catkin_python_setup()

generate_messages(
  DEPENDENCIES
  opencv_apps
  std_msgs
  )

catkin_package(CATKIN_DEPENDS message_runtime rospy) 

roslaunch_add_file_check(launch)


# install(
#   DIRECTORY config launch
#   PROGRAMS bin/featurization
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )




