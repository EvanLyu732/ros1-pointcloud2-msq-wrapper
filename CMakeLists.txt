cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(zvision_msq_wrapper)

option(BUILD_EXAMPLE "building example node" ON)

set(CMAKE_VERBOSE_MAKEFILE FALSE)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    roscpp
)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCE_DIR}/src/entry.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${catkin_LIBRARIES}
)

if(BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

# Install begin
include(GNUInstallDirs)