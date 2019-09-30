cmake_minimum_required(VERSION 2.8.3)
project(phantom_omni)

find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

add_message_files(FILES
  LockState.msg PhantomButtonEvent.msg OmniFeedback.msg)

generate_messages(DEPENDENCIES
  std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
  message_runtime roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(omni src/omni.cpp)

target_link_libraries(omni HD HDU rt ncurses ${catkin_LIBRARIES})


