# Minimum required version of CMake
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

# Add all source files from the src directory excluding main.cpp
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Create the main executable
add_executable(${PROJECT_NAME} ${SOURCES} ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Link OpenCV to the main executable
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${OpenCV_LIBS})


# -----
# TESTS
# -----
# Add the test executable for DataLoader
add_executable(test_dataloader ${CMAKE_SOURCE_DIR}/tests/test_dataloader.cpp ${SOURCES})

# Link OpenCV to the test executable
target_link_libraries(test_dataloader ${OPENGL_LIBRARIES} ${OpenCV_LIBS})
# -----