# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ( "SquashBallTracker" )

set( OpenCV_DIR E:/Documents/opencv/build/install/x64/vc16/lib )

find_package( OpenCV REQUIRED )

add_library(
	ObjectTracker
	./SquashBallTracker/Tracker.cpp
	./SquashBallTracker/Tracker.hpp
)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( SquashBallTracker SquashBallTracker/main.cpp )

target_link_libraries( SquashBallTracker ${OpenCV_LIBS} ObjectTracker)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
