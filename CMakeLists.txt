cmake_minimum_required(VERSION 2.8.3)
project(tiago_wbc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
set(CATKIN_COMPONENTS pal_wbc_controller
    roscpp
    wbc_tasks
    pal_wbc_utils
    pluginlib)

find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_COMPONENTS}
    )

catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS pal_wbc_controller wbc_tasks pluginlib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME} src/tiago_stacks.cpp
    src/tiago_environment_collision_stack.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

### Install ###
install(TARGETS ${PROJECT_NAME}
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

### Tests ###
if (CATKIN_ENABLE_TESTING)

    set(TEST_REQUIRED_CATKIN_COMPONENTS
        tiago_controller_configuration
        eigen_checks
        pal_ros_utils
        smach_c_wbc_states
        smach_c
        ${REQUIRED_CATKIN_COMPONENTS})

    find_package(rostest REQUIRED)
    find_package(catkin REQUIRED COMPONENTS ${TEST_REQUIRED_CATKIN_COMPONENTS})

    include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(push_tasks_test test/push_tasks_test.test test/push_tasks_test.cpp)
    target_link_libraries(push_tasks_test ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(push_tasks_test ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(move_tip_smach_c_test test/move_tip_smach_c_test.test test/move_tip_smach_c_test.cpp)
    target_link_libraries(move_tip_smach_c_test ${catkin_LIBRARIES} ${PROJECT_NAME} )
    add_dependencies(move_tip_smach_c_test ${catkin_EXPORTED_TARGETS})

    add_rostest_gtest(pop_bookkept_test test/pop_bookkept_test.test test/pop_bookkept_test.cpp)
    target_link_libraries(pop_bookkept_test ${catkin_LIBRARIES} ${PROJECT_NAME} )
    add_dependencies(pop_bookkept_test ${catkin_EXPORTED_TARGETS})

endif(CATKIN_ENABLE_TESTING)
