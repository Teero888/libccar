cmake_minimum_required(VERSION 3.10)

project(benchmark)

add_executable(benchmark benchmark.c)

target_include_directories(benchmark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_link_libraries(benchmark m)
