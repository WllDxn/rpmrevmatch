cmake_minimum_required(VERSION 3.10)
project(RPMRevMatch)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "C:/local/boost_1_88_0")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "C:/local/boost_1_88_0lib")

find_package(Boost REQUIRED COMPONENTS system)

# Add executable
add_executable(RPMRevMatch main.cpp)

target_include_directories(RPMRevMatch PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(RPMRevMatch PRIVATE ${Boost_LIBRARIES})