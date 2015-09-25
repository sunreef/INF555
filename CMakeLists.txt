cmake_minimum_required(VERSION 3.3)
project(INF555___Project)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

set(SOURCE_FILES main.cpp particle.cpp particle.h vect.cpp vect.h grid.cpp grid.h kernel.cpp kernel.h cell.cpp cell.h)
add_executable(INF555___Project ${SOURCE_FILES})