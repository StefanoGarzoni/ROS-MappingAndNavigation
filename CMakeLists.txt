cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        tf
        message_filters
        actionlib
        move_base_msgs
        map_server
        stage_ros
        rviz
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        nav_msgs
        tf
        message_filters
        actionlib
        move_base_msgs
        map_server
        stage_ros
        rviz
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_broadcaster           src/tf_broadcaster.cpp)
add_executable(scan_merger              src/scan_merger.cpp)
add_executable(occupancy_grid_mapping   src/occupancy_grid_mapping.cpp)
add_executable(goal_publisher            src/goal_publisher.cpp)

target_link_libraries(tf_broadcaster           ${catkin_LIBRARIES})
target_link_libraries(scan_merger              ${catkin_LIBRARIES})
target_link_libraries(occupancy_grid_mapping   ${catkin_LIBRARIES})
target_link_libraries(goal_publisher            ${catkin_LIBRARIES})