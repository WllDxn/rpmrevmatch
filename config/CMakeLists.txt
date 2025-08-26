add_library(config "")
target_sources(config PUBLIC ${CMAKE_CURRENT_LIST_DIR}/configuration.hpp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/configuration.cpp)
target_include_directories(config PUBLIC ${CMAKE_CURRENT_LIST_DIR})