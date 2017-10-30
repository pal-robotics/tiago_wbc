add_executable(pushTaskExampleTiago examples/pushTaskExampleTiago.cpp)
add_dependencies(pushTaskExampleTiago pal_wbc_controller_generate_messages_cpp)
target_link_libraries(pushTaskExampleTiago ${catkin_LIBRARIES})

add_executable(pushTaskExampleTiagoROS examples/pushTaskExampleTiagoROS.cpp)
add_dependencies(pushTaskExampleTiagoROS pal_wbc_controller_generate_messages_cpp)
target_link_libraries(pushTaskExampleTiagoROS ${catkin_LIBRARIES})

### Install ###
install(TARGETS pushTaskExampleTiago
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
