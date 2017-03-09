cmake_minimum_required(VERSION 2.8.3)
project(dino_map)

find_package(catkin REQUIRED COMPONENTS roslib roscpp  
              std_msgs dino_nav)

find_package(PkgConfig)
pkg_search_module(ALLEGRO5 REQUIRED allegro-5.0)
pkg_search_module(ALLEGRO_PRIMITIVES REQUIRED allegro_primitives-5.0)
pkg_search_module(ALLEGRO_TTF REQUIRED allegro_ttf-5.0)

#generate_dynamic_reconfigure_options(
# cfg/Dinonav.cfg
#)

catkin_package(
    #CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
    #DEPENDS TinyXML
)


include_directories(include ${catkin_INCLUDE_DIRS})

message(WARNING "cacca" ${dino_nav})

add_definitions("-std=c++11")


add_executable(dinomap src/main.cpp ../dino_nav/src/viz.cpp ../dino_nav/src/common.cpp)
target_link_libraries(dinomap ${catkin_LIBRARIES} ${ALLEGRO5_LIBRARIES} ${ALLEGRO_PRIMITIVES_LIBRARIES}
                              ${ALLEGRO_TTF_LIBRARIES})
