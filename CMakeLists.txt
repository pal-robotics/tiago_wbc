cmake_minimum_required(VERSION 2.8.3)
project(tiago_wbc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pal_wbc_controller
  roscpp
  wbc_tasks
  tiago_description
)

catkin_package(
  CATKIN_DEPENDS pal_wbc_controller wbc_tasks pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(tiago_wbc src/tiago_stacks.cpp
                      src/tiago_rcp_stack.cpp)
                     # src/tiago_environment_collision_stack.cpp)
target_link_libraries(tiago_wbc ${catkin_LIBRARIES})

add_subdirectory(src/rcp)

### Install ###
install(TARGETS tiago_wbc
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tiago_wbc_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
