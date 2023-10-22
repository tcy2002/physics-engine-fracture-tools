cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(PhysEngine)

SET(CMAKE_CXX_STANDARD 17)

include_directories(. eigen)

add_subdirectory(common)

link_libraries(EngineCommon)

add_executable(test main.cpp)
