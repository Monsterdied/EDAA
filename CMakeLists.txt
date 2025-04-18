cmake_minimum_required(VERSION 3.10)

# Project name
project(EDAA_Project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(.)

# Add source files
set(SOURCES
    src/main.cpp
    src/Node.cpp
    src/Edge.cpp
    src/Graph.cpp
    src/Manager.cpp
)

# Add the executable
add_executable(EDAA_Project ${SOURCES})