cmake_minimum_required (VERSION 2.8)

set (CMAKE_CXX_STANDARD 11)

project(vision)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "/3314/ntcore/include/" "/3314/ntcore/wpiutil/include/" "/usr/include/gstreamer-1.0/" "/usr/include/glib-2.0/" "/usr/lib/aarch64-linux-gnu/glib-2.0/include/" "/usr/lib/aarch64-linux-gnu/gstreamer-1.0/include/")

add_executable (viz vision.cpp vision.hpp cap_gstreamer.cpp cap_gstreamer.hpp cvconfig.h precomp.hpp)

add_executable (cal calibrate.cpp vision.hpp cap_gstreamer.cpp cap_gstreamer.hpp cvconfig.h precomp.hpp)

find_library(LibNT ntcore "/3314/ntcore/")

find_library(GST HINT gst PATHS "/usr/lib/aarch64-linux-gnu/gstreamer-1.0/")

target_link_libraries(viz ${OpenCV_LIBS} ${LibNT} "/usr/lib/aarch64-linux-gnu/libgstreamer-1.0.so" "/usr/lib/aarch64-linux-gnu/libgobject-2.0.so" "/usr/lib/aarch64-linux-gnu/libglib-2.0.so" "/usr/lib/aarch64-linux-gnu/libgstapp-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstriff-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstbase-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstvideo-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstpbutils-1.0.so")

target_link_libraries(cal ${OpenCV_LIBS} ${LibNT} "/usr/lib/aarch64-linux-gnu/libgstreamer-1.0.so" "/usr/lib/aarch64-linux-gnu/libgobject-2.0.so" "/usr/lib/aarch64-linux-gnu/libglib-2.0.so" "/usr/lib/aarch64-linux-gnu/libgstapp-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstriff-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstbase-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstvideo-1.0.so" "/usr/lib/aarch64-linux-gnu/libgstpbutils-1.0.so")
