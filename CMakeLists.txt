cmake_minimum_required(VERSION 2.8.3)
project(tiago_wbc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    pal_wbc_controller
    roscpp
    wbc_tasks
    pal_wbc_utils
    )

catkin_package(
    CATKIN_DEPENDS pal_wbc_controller wbc_tasks pluginlib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME} src/tiago_stacks.cpp
							src/tiago_environment_collision_stack.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_interactive_marker_task src/push_interactive_marker_task.cpp)
target_link_libraries(push_interactive_marker_task ${PROJECT_NAME} ${catkin_LIBRARIES})

### Install ###
install(TARGETS ${PROJECT_NAME} push_interactive_marker_task
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
