cmake_minimum_required(VERSION 2.8.3)
project(dino_nav)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs genmsg race dynamic_reconfigure)

find_package(message_generation)

generate_dynamic_reconfigure_options(
  cfg/Dinonav.cfg
)

catkin_package(
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dinonav src/dinonav.cpp src/pathfind.cpp)
target_link_libraries(dinonav  ${catkin_LIBRARIES})
# make sure configure headers are built before any node using them
add_dependencies(dinonav ${PROJECT_NAME}_gencfg)

