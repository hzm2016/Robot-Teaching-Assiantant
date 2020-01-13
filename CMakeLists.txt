cmake_minimum_required(VERSION 3.1)

# Project name
project (ATIDAQ)

# Make sure to link against libpthread
find_package(Threads REQUIRED)

# Redirect binaries to bin folder.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Add sub directories
add_subdirectory(ATIDAQ)
add_subdirectory(Model526)
add_subdirectory(apps)
