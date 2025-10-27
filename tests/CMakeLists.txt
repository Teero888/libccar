cmake_minimum_required(VERSION 3.10)

project(libccar-tests)

add_library(libccar_impl libccar_impl.c)

target_include_directories(libccar_impl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)

add_subdirectory(benchmark)
add_subdirectory(example)
add_subdirectory(max_speed)