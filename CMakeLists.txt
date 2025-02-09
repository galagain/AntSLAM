cmake_minimum_required(VERSION 3.10)

# Project name
project(AntSLAM)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include header files from the include directory
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find the system OpenCV package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add FreeGLUT as a submodule
set(FREEGLUT_DIR ${CMAKE_SOURCE_DIR}/external/freeglut)

# Check if FreeGLUT exists in the submodules
if(NOT EXISTS ${FREEGLUT_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "FreeGLUT submodule not found! Make sure to run `git submodule update --init --recursive`.")
endif()

# Include FreeGLUT
add_subdirectory(${FREEGLUT_DIR})
include_directories(${FREEGLUT_DIR}/include)

# Find OpenGL
find_package(OpenGL REQUIRED)
if (OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIRS})
    link_libraries(${OPENGL_LIBRARIES})
else()
    message(FATAL_ERROR "OpenGL not found!")
endif()

# Add Eigen as a submodule
set(EIGEN_DIR ${CMAKE_SOURCE_DIR}/external/eigen)
include_directories(${EIGEN_DIR})
message(STATUS "Eigen included from: ${EIGEN_DIR}")

# Add all source files from the src directory excluding main.cpp
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Create the main executable
add_executable(${PROJECT_NAME} ${SOURCES} ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Link OpenCV, FreeGLUT, and Eigen to the main executable
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} freeglut_static)

# -----
# TESTS
# -----
# Add the test executable for DataLoader
add_executable(test_dataloader ${CMAKE_SOURCE_DIR}/tests/test_dataloader.cpp ${SOURCES})

# Link OpenCV, FreeGLUT, and Eigen to the test executable
target_link_libraries(test_dataloader ${OpenCV_LIBS} freeglut_static)
