cmake_minimum_required(VERSION 2.8)
project( BoilerMonitorCV )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( BoilerMonitorCV BoilerMonitorCV.cpp )
target_link_libraries( BoilerMonitorCV libb64.a ${OpenCV_LIBS} )

