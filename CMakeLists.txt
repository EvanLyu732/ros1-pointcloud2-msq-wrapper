cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(zvision_msq_wrapper)

option(BUILD_EXAMPLE OFF)

set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    roscpp
)

# find_package(Boost REQUIRED COMPONENTS
#     system
# )

add_library(${PROJECT_NAME} STATIC 
    ${PROJECT_SOURCE_DIR}/src/entry.cc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${catkin_LIBRARIES}
    # ${Boost_LIBRARIES}
)

message(VAR ${catkin_INCLUDE_DIRS})


if(BUILD_EXAMPLE)
    include(${PROJECT_SOURCE_DIR}/example)
endif()

# Install begin
include(GNUInstallDirs)