新しいパッケージの作成
cv_bridgeをテストするための新しいパッケージ（cvTest）を以下のように作成します。
$ cd ~/catkin_ws/src
$ catkin_create_pkg cvTest sensor_msgs cv_bridge roscpp rospy std_msgs image_transport

remap タグにより、変換を行ないます。
CMakeLists.txtの修正
~/catkin_ws/src/cvTest/CMakeLists.txt の最後に以下の4行を加えて下さい。

find_package( OpenCV REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${OpenCV_LIBRARIES} ${catkin_LIBRARIES}  )
パッケージをmakeして、実行する。
$ cd ~/catkin_ws
$ catkin_make

さらに別のターミナルウィンドウで
image_converter.cppを修正した場合は、
$ rosrun cv_test image_converter