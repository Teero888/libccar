cmake_minimum_required(VERSION 3.10)

project(max_speed)

add_executable(max_speed main.c)

target_include_directories(max_speed PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_link_libraries(max_speed m)
