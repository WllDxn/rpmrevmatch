cmake_minimum_required(VERSION 3.10)
project(RPMRevMatch)
add_compile_definitions(_WIN32_WINNT=0x0601)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find Boost
set(BOOST_ROOT "C:/local/boost_1_88_0")

# Find Boost headers only (you only need headers for Asio unless you're using Boost::System)
find_package(Boost REQUIRED)

# Add subdirectories for internal libraries
add_subdirectory(utils)
add_subdirectory(config)
add_subdirectory(io)

# Add your executable
add_executable(RPMRevMatch src/main.cpp)
target_sources(RPMRevMatch PRIVATE
    utils/signal_handler.cpp
)

# Link libraries
target_link_libraries(RPMRevMatch PRIVATE utils io config ${Boost_LIBRARIES})

# Add Boost headers
target_include_directories(RPMRevMatch PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(RPMRevMatch PRIVATE "C:/local/boost_1_88_0")
