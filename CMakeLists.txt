cmake_minimum_required(VERSION 2.8.3)
project(hand_detector)

find_package(catkin REQUIRED
	pcl_conversions
	pcl_ros
	roscpp
	sensor_msgs
	geometry_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hand_detector src/detector.cpp)
target_link_libraries(hand_detector ${catkin_LIBRARIES})
add_dependencies(hand_detector hand_detector_generate_messages_cpp)
