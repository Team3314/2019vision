cmake_minimum_required (VERSION 2.8)

set (CMAKE_CXX_STANDARD 11)

project(vision)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "/3314/ntcore/include/" "/3314/ntcore/wpiutil/include/")

add_executable (viz vision.cpp)

find_library(LibNT ntcore "/3314/ntcore/")

target_link_libraries(viz ${OpenCV_LIBS} ${LibNT})
