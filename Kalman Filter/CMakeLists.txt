# CMakeList.txt : CMake project for Kalman Filter, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

#file(GLOB SOURCES
 #   "Kalman Filter.h"
  #  "Kalman Filter.cpp"
   # "Test.cpp"
#)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Add source to this project's executable.
add_executable (CMakeTarget "Test.cpp" "KalmanFilter.cpp")
# TODO: Add tests and install targets if needed.
target_link_libraries (CMakeTarget Eigen3::Eigen)
