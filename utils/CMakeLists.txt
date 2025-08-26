add_library(utils INTERFACE)
target_include_directories(utils INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_sources(utils
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/error_handling.hpp
        ${CMAKE_CURRENT_LIST_DIR}/signal_handler.hpp
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/signal_handler.cpp
)
