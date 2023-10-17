cmake_minimum_required(VERSION 3.21)
project(test)

set(CMAKE_CXX_STANDARD 14)

add_executable(test main.cpp general.h test_vector.h hash_vector.h triangle_manager.h math_utils.h vector3.h real.h voronoi_calculator.h fracture_calculator.h)
