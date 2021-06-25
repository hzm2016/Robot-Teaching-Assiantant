###################################################################
# Data file macros generator:
###################################################################


###################################################################
# Control program - basic functions:
###################################################################

# add_executable(src
#    main.cpp)  

# pybind11_add_module(motor_control main.cpp)

link_libraries(
s526_lib
rtsetup_lib
rtutils_lib
thread_service_lib
daq_lib
socketcan_lib
gyems_can_lib
${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(motor_control LINK_PUBLIC
m
s526_lib
rtsetup_lib
rtutils_lib
thread_service_lib
daq_lib
socketcan_lib
gyems_can_lib
${CMAKE_THREAD_LIBS_INIT})

target_include_directories(motor_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})