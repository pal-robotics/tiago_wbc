#add_executable(listStackExample examples/listStackExample.cpp)
#target_link_libraries(listStackExample ${catkin_LIBRARIES})

#add_executable(pushTaskExample examples/pushTaskExample.cpp)
#target_link_libraries(pushTaskExample ${catkin_LIBRARIES})

add_executable(pushTaskExampleTiago examples/pushTaskExampleTiago.cpp)
add_dependencies(pushTaskExampleTiago pal_wbc_controller_generate_messages_cpp)
target_link_libraries(pushTaskExampleTiago ${catkin_LIBRARIES})

#add_executable(pushPopTasksExample examples/pushPopTasksExample.cpp)
#target_link_libraries(pushPopTasksExample ${catkin_LIBRARIES})

