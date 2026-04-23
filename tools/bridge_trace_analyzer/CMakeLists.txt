cmake_minimum_required(VERSION 3.16)

project(bridge_trace_analyzer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(bridge_trace_analyzer
  src/main.cpp
)

target_compile_options(bridge_trace_analyzer PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
