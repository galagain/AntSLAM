# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Project name
project(AntSLAM)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include header files from the include directory
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add all source files from the src directory
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})