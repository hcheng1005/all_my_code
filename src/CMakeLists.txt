cmake_minimum_required(VERSION 2.8.3)
project(demo_ros)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

###########
## Build ##
###########
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(PCL REQUIRED)
set(Eigen_INCLUDE_DIRS "usr/include/eigen3")
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
)

# robot_demo(EKF UKF demo)
# add_library(robot_demo src/Robot1/main.cpp)
# target_link_libraries(robot_demo ${catkin_LIBRARIES})
# add_executable(robot_demo_node src/Robot1/main.cpp)
# target_link_libraries(robot_demo_node robot_demo ${catkin_LIBRARIES})

# # robot_demo(EKF UKF demo)
# add_library(radar_demo src/Robot1/main2.cpp)
# target_link_libraries(radar_demo ${catkin_LIBRARIES})
# add_executable(radar_demo_node src/Robot1/main2.cpp)
# target_link_libraries(radar_demo_node radar_demo ${catkin_LIBRARIES})

# simple_trace_demo
add_library(simple_trace_demo src/SimpleTracker/simpletrack_node.cpp)
target_link_libraries(simple_trace_demo ${catkin_LIBRARIES})
add_executable(simple_trace_demo_node src/SimpleTracker/simpletrack_node.cpp include/distance/iou.cpp)
target_link_libraries(simple_trace_demo_node simple_trace_demo ${catkin_LIBRARIES})

# # assign_demo
# add_library(assign_demo src/Assignment/assign_demo.cpp )
# target_link_libraries(assign_demo ${catkin_LIBRARIES})
# add_executable(assign_demo_node src/Assignment/assign_demo.cpp include/assignment/HungarianAlg.cpp include/assignment/auction.cpp include/assignment/Lapjv.cpp)
# target_link_libraries(assign_demo_node assign_demo ${catkin_LIBRARIES})

# add_library(kbest_demo src/Assignment/kbest_demo.cpp )
# target_link_libraries(kbest_demo ${catkin_LIBRARIES})
# add_executable(kbest_demo_node src/Assignment/kbest_demo.cpp  include/assignment/auction.cpp include/assignment/Lapjv.cpp)
# target_link_libraries(kbest_demo_node kbest_demo ${catkin_LIBRARIES})