cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME ball_throwing)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)


set(PROJECT_DEPENDENCIES 
  roscpp  
  std_msgs
  sensor_msgs
  mav_msgs
  tf2_ros
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PROJECT_DEPENDENCIES}
)

find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
	include
  include/${PROJECT_NAME}
)

set(SOURCE_FILES
  src/ball_throwing.cpp
)

add_executable(${PROJECT_NAME}_node src/ball_throwing_node.cpp ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
