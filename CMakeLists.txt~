cmake_minimum_required(VERSION 2.8.3)
project(kylinbot_core)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  CBUS.msg
  Kylin.msg
  PosCalib.msg
  Sonar.msg
  VirtualRC.msg
  ZGyro.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kylinbot_core
  CATKIN_DEPENDS message_runtime
  DEPENDS system_lib
)

find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Release)

FILE(GLOB sourcefiles "src/asp/src/*.c" "src/uart/src/*.cpp")
include_directories(/usr/local/include include include/asp/inc include/uart/inc ${catkin_INCLUDE_DIRS})

link_directories(/usr/local/lib /opt/ros/indigo/lib)

add_executable(kylinbot_msg_puller ${sourcefiles} src/kylinbot_msg_puller.cpp)
target_link_libraries(kylinbot_msg_puller  ${catkin_LIBRARIES} tf)
add_dependencies(kylinbot_msg_puller kylinbot_core_generate_messages_cpp)

add_executable(kylinbot_msg_pusher ${sourcefiles} src/kylinbot_msg_pusher.cpp)
target_link_libraries(kylinbot_msg_pusher  ${catkin_LIBRARIES})
add_dependencies(kylinbot_msg_puller kylinbot_core_generate_messages_cpp)

#add_executable(MarkerDetection src/viz/src/MarkerDetection.cpp )
#target_link_libraries(MarkerDetection ${OpenCV_LIBS} ${catkin_LIBRARIES} )

add_executable(kylinbot_logic_controller src/kylinbot_logic_controller.cpp ${sourcefiles} )
target_link_libraries(kylinbot_logic_controller ${catkin_LIBRARIES})
add_dependencies(kylinbot_logic_controller move_base_msgs)

add_executable(kylinbot_rc_playback ${sourcefiles} src/kylinbot_rc_playback.cpp)
target_link_libraries(kylinbot_rc_playback  ${catkin_LIBRARIES})
add_dependencies(kylinbot_rc_playback kylinbot_core_generate_messages_cpp)

