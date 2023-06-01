cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_registration)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_registration cloud_registration.cpp)
target_link_libraries (cloud_registration ${PCL_LIBRARIES})