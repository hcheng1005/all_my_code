# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)
 
# 项目信息
project (multi_sensor_mot)

# For Linux
find_package(OpenCV)
FIND_PACKAGE(OpenCV REQUIRED PATHS "/usr/local/include/opencv2")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

 
# 指定生成目标
add_executable(multi_sensor_mot ${DIR_SRCS})

target_link_libraries(multi_sensor_mot ${OpenCV_LIBS})
