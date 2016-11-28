cmake_minimum_required(VERSION 2.8.3)
project(dino_nav)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs genmsg race dynamic_reconfigure)

find_package(message_generation)

find_package(PkgConfig)
pkg_search_module(ALLEGRO5 allegro-5.0)
pkg_search_module(ALLEGRO_PRIMITIVES allegro_primitives-5.0)
pkg_search_module(ALLEGRO_TTF allegro_ttf-5.0)

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

if(ALLEGRO5_FOUND)
    add_executable(viewer src/viewer.cpp)
    target_link_libraries(viewer  ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                                  ${ALLEGRO_TTF_LIBRARIES})
endif()