cmake_minimum_required(VERSION 3.10)

project(example)

add_executable(example example.c)

target_include_directories(example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_link_libraries(example m libccar_impl)
