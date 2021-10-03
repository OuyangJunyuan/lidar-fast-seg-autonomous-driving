cmake_minimum_required(VERSION 2.8.12)
project(point)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
set(CMAKE_BUILD_TYPE "Release")
find_package(catkin COMPONENTS
        roscpp
        sensor_msgs
        pcl_conversions
        REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

add_definitions( ${PCL_DEFINITIONS} )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )

add_executable(main main.cpp CVC_cluster.cpp)
target_link_libraries(main ${PCL_LIBRARIES}
        ${catkin_LIBRARIES})
