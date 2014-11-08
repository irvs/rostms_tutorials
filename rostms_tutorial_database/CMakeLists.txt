cmake_minimum_required(VERSION 2.8.3)
project(rostms_tutorial_database)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tms_msg_db
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES rostms_tutorial_database
  CATKIN_DEPENDS roscpp std_msgs tms_msg_db
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rostms_tutorial_db_writer src/ros_tutorial_db_writer.cpp)
add_dependencies(rostms_tutorial_db_writer tms_msg_db_generate_messages_cpp)
target_link_libraries(rostms_tutorial_db_writer ${catkin_LIBRARIES} mysqlclient)

add_executable(rostms_tutorial_db_reader src/ros_tutorial_db_reader.cpp)
add_dependencies(rostms_tutorial_db_reader tms_msg_db_generate_messages_cpp)
target_link_libraries(rostms_tutorial_db_reader ${catkin_LIBRARIES} mysqlclient)
