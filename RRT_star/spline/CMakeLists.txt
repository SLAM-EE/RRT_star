cmake_minimum_required(VERSION 2.8)
project(spline)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_SPLINE
    include/Bezier.h
    include/BSpline.h
    include/CatmullRom.h
    include/Curve.h
    include/Vector.h)
set(SRCS_SPLINE
  src/Bezier.cpp
  src/BSpline.cpp
  src/CatmullRom.cpp
  src/Curve.cpp
  src/Vector.cpp)


find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(SPLINE SHARED ${SRCS_SPLINE})
target_link_libraries(SPLINE ${OpenCV_LIBS})

