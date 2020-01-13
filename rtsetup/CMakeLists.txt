# Create a library
add_library (rtthread_lib rtsetup.c)

target_include_directories(rtthread_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
