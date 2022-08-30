cmake_minimum_required(VERSION 2.8.3)
project(gutea_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  FILES
  ClassificationResult.msg
)

add_service_files(
  FILES
  DoSpeech.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime) 

include_directories()
