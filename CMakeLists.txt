cmake_minimum_required(VERSION 3.0.2)
project(navigator)

find_package(catkin REQUIRED COMPONENTS
  amcl
  dwa_planner
  geometry_msgs
  roomba_500driver_meiji
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/navigator.cpp src/navigator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
