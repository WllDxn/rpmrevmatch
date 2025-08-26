add_library(io "")

# Boost paths (if needed)
SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "C:/local/boost_1_88_0")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "C:/local/boost_1_88_0lib")


set_target_properties(io PROPERTIES LINKER_LANGUAGE CXX)

# Link utils so io can use its headers
target_link_libraries(io PRIVATE utils)

# Add all source and header files for io
target_sources(io
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/csv_writer.cpp
        ${CMAKE_CURRENT_LIST_DIR}/elm327.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/csv_writer.hpp
        ${CMAKE_CURRENT_LIST_DIR}/elm327.hpp
)

# Make the io directory available for includes as <io/filename.hpp>
target_include_directories(io PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(io PRIVATE utils)
target_include_directories(io PUBLIC ${Boost_INCLUDE_DIRS})
