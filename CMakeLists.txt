cmake_minimum_required(VERSION 3.10)
project(eliminar_suelo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(eliminar_suelo main.cpp)
target_link_libraries(eliminar_suelo ${PCL_LIBRARIES})
