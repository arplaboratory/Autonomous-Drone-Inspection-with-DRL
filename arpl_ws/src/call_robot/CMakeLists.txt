cmake_minimum_required(VERSION 2.8)
project(call_robot)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)
add_service_files(DIRECTORY srv FILES CallRobot.srv Vec4.srv)
#catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs)
catkin_package()
install(PROGRAMS scripts/call_robot_srv DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
