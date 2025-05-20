cmake_minimum_required(VERSION 3.0.2)
project(plc_message_parser)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  dynamic_reconfigure
  serial
  roscpp_serialization
)
  find_library(YAML_CPP_LIB yaml-cpp)    

## Add message files
add_message_files(
  FILES
  Heartbeat.msg
  PickPlace.msg
  Acknowledgement.msg
  JobTwistlock.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs dynamic_reconfigure serial roscpp_serialization
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Add the executable
add_executable(plc_message_parser_node src/plc_message_parser.cpp)
add_executable(plc_driver_node src/plc_driver.cpp)

## Link the executable with the required libraries
target_link_libraries(plc_driver_node ${catkin_LIBRARIES} ${YAML_CPP_LIB})
target_link_libraries(plc_message_parser_node ${catkin_LIBRARIES})


## Add dependencies to generated messages
add_dependencies(plc_message_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plc_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


