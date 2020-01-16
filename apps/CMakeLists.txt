add_executable(ftconvert ftconvert.cpp)
add_executable(calinfo calinfo.cpp)
add_executable(dynamo_test cyclic_task.cpp)

target_link_libraries(ftconvert LINK_PUBLIC daq_lib m)
target_link_libraries(calinfo LINK_PUBLIC daq_lib m)
target_link_libraries(dynamo_test LINK_PUBLIC
  daq_lib
  m
  s526_lib
  innfos_can_lib
  rtthread_lib
  rtutils_lib
  ${CMAKE_THREAD_LIBS_INIT})
