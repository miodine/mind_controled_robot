cmake_minimum_required(VERSION 3.0.2)
project(mcr_messages)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
    mcr_datastream.msg
    mcr_predictions.msg
    mcr_intention.msg
    mcr_control_monit.msg
)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
