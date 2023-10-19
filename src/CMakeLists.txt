cmake_minimum_required(VERSION 2.8.7)
project(DemoList)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

###########
## Build ##
###########

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf
)


set(PCL_INCLUDE_DIRS "usr/include/pcl-1.12/pcl")
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
set(OpenCV_INCLUDE_DIRS "usr/local/include/opencv2")
set(Eigen_INCLUDE_DIRS "usr/include/eigen3")

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
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

#---------------------------------------------------------------------------------

# # simple_trace_demo
# add_library(simple_trace_demo src/SimpleTracker/main.cpp)
# target_link_libraries(simple_trace_demo ${catkin_LIBRARIES})
# add_executable(simple_trace_demo_node src/SimpleTracker/main.cpp)
# target_link_libraries(simple_trace_demo_node simple_trace_demo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_library(sub_demo src/SimpleTracker/simpletrack_node.cpp)
# target_link_libraries(sub_demo ${catkin_LIBRARIES})
# add_executable(simple_trace_sub_node src/SimpleTracker/simpletrack_node.cpp include/distance/iou.cpp src/SimpleTracker/simpletracker.cpp)
# target_link_libraries(simple_trace_sub_node sub_demo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#---------------------------------------------------------------------------------

#radar_tracking_demo
add_library(radar_tracking_demo src/RadarDemo/main.cpp)
target_link_libraries(radar_tracking_demo ${catkin_LIBRARIES})
add_executable(radar_tracking_demo_node src/RadarDemo/main.cpp src/RadarDemo/radarTracker.cpp src/RadarDemo/DBSCAN.cc)
target_link_libraries(radar_tracking_demo_node radar_tracking_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#---------------------------------------------------------------------------------

# # assign_demo
# add_library(assign_demo src/Assignment/assign_demo.cpp )
# target_link_libraries(assign_demo ${catkin_LIBRARIES})
# add_executable(assign_demo_node src/Assignment/assign_demo.cpp include/assignment/HungarianAlg.cpp include/assignment/auction.cpp include/assignment/Lapjv.cpp)
# target_link_libraries(assign_demo_node assign_demo ${catkin_LIBRARIES})

# add_library(kbest_demo src/Assignment/kbest_demo.cpp )
# target_link_libraries(kbest_demo ${catkin_LIBRARIES})
# add_executable(kbest_demo_node src/Assignment/kbest_demo.cpp  include/assignment/auction.cpp include/assignment/Lapjv.cpp)
# target_link_libraries(kbest_demo_node kbest_demo ${catkin_LIBRARIES})


# DATMO demo
#MultiThreading Library
find_package(OpenMP REQUIRED)
OPTION (USE_OpenMP "Use OpenMP" OFF)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  visualization_msgs
  cmake_modules REQUIRED
  message_generation
)

add_message_files(
  FILES
  Track.msg
  TrackArray.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

add_executable(datmo src/DATMO/src/main.cpp src/DATMO/src/l_shape_tracker.cpp src/DATMO/src/kalman-cpp/kalman.cpp src/DATMO/src/datmo.cpp src/DATMO/src/cluster.cpp)
add_dependencies(datmo ${catkin_EXPORTED_TARGETS} datmo_generate_messages_cpp)
target_link_libraries(datmo ${catkin_LIBRARIES})