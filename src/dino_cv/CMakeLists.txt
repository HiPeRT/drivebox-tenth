cmake_minimum_required(VERSION 2.8.3)
project(dino_cv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cv_test2
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package( OpenCV REQUIRED )

find_package(PkgConfig)
pkg_search_module(ZBAR zbar)

add_definitions("-std=c++11")

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
add_executable(qr_detector src/qr_detector.cpp)
target_link_libraries(qr_detector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${ZBAR_LIBRARIES})

add_executable(speed_detector src/speed_detector.cpp)
target_link_libraries(speed_detector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

