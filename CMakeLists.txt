#---------------------------------------------------
# CMAKE VERSION
# --------------------------------------------------
cmake_minimum_required(VERSION 3.12)

# --------------------------------------------------
# PROJECT NAME AND VERSION
# --------------------------------------------------
project(SquashBallTracker VERSION 1.0)

# --------------------------------------------------
# GENERATE COMPILATION DATABASE (FOR YCM)
# --------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------------------------------
# BUILD SHARED LIBRARIES
# --------------------------------------------------
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# --------------------------------------------------
# CONFIGURATION HEADER
# --------------------------------------------------
#configure_file(include/Config.h.in 
#    ${CMAKE_CURRENT_SOURCE_DIR}/include/Config.h)

# --------------------------------------------------
# EXECUTABLE
# --------------------------------------------------
add_executable(SquashBallTracker
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/SBT2.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/App.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/App.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ImageProcessor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ImageProcessor.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ObjectDetector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ObjectDetector.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ProjectileMaths.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SquashBallTracker/ProjectileMaths.hpp"
    )

# --------------------------------------------------
# TARGET INCLUDE DIRECTORIES
# --------------------------------------------------
find_package(OpenCV REQUIRED)

if(${OpenCV_FOUND})
    message("OpenCV VERSION : ${OpenCV_VERSION}")

    target_include_directories(SquashBallTracker PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${OpenCV_INCLUDE_DIRS}")

    target_link_libraries(SquashBallTracker ${OpenCV_LIBRARIES})

    message("OpenCV Libraries Path: ${OpenCV_INCLUDE_DIRS}")
    
endif()
# --------------------------------------------------
# SET OUTPUT DIRECTORIES
# --------------------------------------------------
set_target_properties(SquashBallTracker
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/$<CONFIG>")
     
# --------------------------------------------------
# COMPILER FLAGS
# --------------------------------------------------
target_compile_features(SquashBallTracker PUBLIC cxx_std_20)

# add compiler warning flags just when building this project
# via the BUILD_INTERFACE generator expression
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

set(gcc_flags "-Wall;-Wextra;-Wshadow;-Wshadow;-Wformat=2")
set(msvc_flags "-W3")

target_compile_options(SquashBallTracker PRIVATE
   "$<${gcc_like_cxx}:$<BUILD_INTERFACE:${gcc_flags}>>"
   "$<${msvc_cxx}: $<BUILD_INTERFACE:${msvc_flags}>>")

