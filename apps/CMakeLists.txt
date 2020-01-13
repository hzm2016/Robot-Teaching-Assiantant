add_executable(ftconvert ftconvert.c)
add_executable(calinfo calinfo.c)
add_executable(dynamo_test cyclic_task.c)

target_link_libraries(ftconvert LINK_PUBLIC daq_lib m)
target_link_libraries(calinfo LINK_PUBLIC daq_lib m)
target_link_libraries(dynamo_test LINK_PUBLIC
  daq_lib m
  s526_lib
  ${CMAKE_THREAD_LIBS_INIT}
  rtthread_lib)
