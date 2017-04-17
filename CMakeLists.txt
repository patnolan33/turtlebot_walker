cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_walker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	move_base_msgs
	sensor_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(
	  walkerTest
	  test/walkerTest.launch
	  test/main.cpp
	  test/walkerTest.cpp
  )
  
  target_link_libraries(
	  walkerTest
	  ${catkin_LIBRARIES}
  )
  
  add_dependencies(
	  walkerTest
	  walker
	  ${catkin_EXPORTED_TARGETS}
  )
endif()
