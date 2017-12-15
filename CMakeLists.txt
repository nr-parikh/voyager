cmake_minimum_required(VERSION 2.8.3)
project(voyager)

#################################
## Dependencies of the project ##
#################################

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation 
  genmsg
)

set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#########################
## Service declaration ##
#########################

add_service_files(
  FILES
  explore.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp std_msgs roscpp rospy sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/voyager.cpp src/laser_scan.cpp src/planner.cpp src/quadrotor.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    voyager_test
    launch/test_voyager.launch
    test/test_${PROJECT_NAME}_node.cpp
    test/test_laser_scan.cpp
    test/test_planner.cpp
    test/test_quadrotor.cpp
    test/test_voyager.cpp
    src/laser_scan.cpp
    src/planner.cpp
    src/quadrotor.cpp

  )

  target_link_libraries(
    voyager_test
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    voyager_test
    ${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
endif()