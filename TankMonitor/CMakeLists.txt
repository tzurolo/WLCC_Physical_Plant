cmake_minimum_required(VERSION 2.8)
project( GaugeReader )
find_package( OpenCV REQUIRED )
find_package( raspicam REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( GaugeReader GaugeReader.cpp )
target_link_libraries( GaugeReader libb64.a ${OpenCV_LIBS} ${raspicam_CV_LIBS} )

