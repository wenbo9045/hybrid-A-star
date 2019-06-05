cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

## C++11
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    )
## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  tf
  std_msgs
  
  INCLUDE_DIRS include
  LIBRARIES hybrid_astar_search
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
)

add_library(hybrid_astar_search
  src/algorithm.cpp
  src/node2d.cpp
  src/node3d.cpp
  src/collisiondetection.cpp
  src/path.cpp
  src/dubins.cpp 
)

add_executable(hybrid_astar src/hybrid_main.cpp ${HEADERS} ${SOURCES})
add_dependencies(hybrid_astar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_astar hybrid_astar_search ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
