cmake_minimum_required(VERSION 3.1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE DEBUG)
endif()

# Project name
project (ATIDAQ LANGUAGES CXX C)

# Other build flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -march=atom -mtune=atom")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DFDEBUG")

# Make sure to link against libpthread
find_package(Threads REQUIRED)

# Redirect binaries to bin folder.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Add sub directories
add_subdirectory(ATIDAQ)
add_subdirectory(Model526)
add_subdirectory(rtsetup)
add_subdirectory(InnfosCAN)
add_subdirectory(apps)
