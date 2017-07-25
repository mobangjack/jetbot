cmake_minimum_required(VERSION 2.8.3)
project(jetbot)

find_package(catkin REQUIRED COMPONENTS
   tf
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_generation
)

add_message_files(
  FILES
  ChassisState.msg
  PantiltState.msg
  CBus.msg
  Bot.msg
  Uwb.msg
  VRC.msg
  VHC.msg
  VDBus.msg
  ZGyro.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
 # jetbot_message_generations_cpp
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

FILE(GLOB sourcefiles "src/asp/src/*.c" "src/uart/src/*.cpp")
include_directories(/usr/local/include include include/asp/inc include/uart/inc ${catkin_INCLUDE_DIRS})

add_executable(jetbot_msg_puller ${sourcefiles} src/jetbot_msg_puller.cpp)
target_link_libraries(jetbot_msg_puller ${catkin_LIBRARIES})

add_executable(jetbot_msg_pusher ${sourcefiles} src/jetbot_msg_pusher.cpp)
target_link_libraries(jetbot_msg_pusher ${catkin_LIBRARIES})

add_executable(jetbot_cmd_vel ${sourcefiles} src/jetbot_cmd_vel.cpp)
target_link_libraries(jetbot_cmd_vel ${catkin_LIBRARIES})

add_executable(jetbot_odo_pub ${sourcefiles} src/jetbot_odo_pub.cpp)
target_link_libraries(jetbot_odo_pub ${catkin_LIBRARIES})



