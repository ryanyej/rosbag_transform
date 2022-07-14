cmake_minimum_required(VERSION 2.8.3)
project(bag_transform)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        roscpp
        pcl_conversions
        pcl_ros
        tf
        rospy
        sensor_msgs
        rosbag
        )

catkin_package(CATKIN_DEPENDS)
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable(bag_trans src/bag_points_trans.cpp)

target_link_libraries(bag_trans
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES}
        )