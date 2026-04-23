cmake_minimum_required(VERSION 3.16)

project(fst_slice_vcd LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(FSTAPI_DIR "" CACHE PATH
    "Directory containing fstapi.h, fstapi.c, fastlz.c, and lz4.c")

if(NOT FSTAPI_DIR AND DEFINED ENV{FSTAPI_DIR} AND NOT "$ENV{FSTAPI_DIR}" STREQUAL "")
  set(FSTAPI_DIR "$ENV{FSTAPI_DIR}")
endif()

if(NOT FSTAPI_DIR AND DEFINED ENV{VERILATOR_ROOT} AND NOT "$ENV{VERILATOR_ROOT}" STREQUAL "")
  set(FSTAPI_DIR "$ENV{VERILATOR_ROOT}/include/gtkwave")
endif()

if(NOT FSTAPI_DIR)
  execute_process(
    COMMAND verilator -V
    OUTPUT_VARIABLE VERILATOR_INFO
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  string(REGEX MATCH "VERILATOR_ROOT[ \t]*=[ \t]*([^\n\r]+)"
    VERILATOR_ROOT_MATCH "${VERILATOR_INFO}")
  if(VERILATOR_ROOT_MATCH)
    set(_verilator_root "${CMAKE_MATCH_1}")
    string(STRIP "${_verilator_root}" _verilator_root)
    set(FSTAPI_DIR "${_verilator_root}/include/gtkwave")
  endif()
endif()

if(NOT FSTAPI_DIR)
  message(FATAL_ERROR
    "Could not determine FST API location. Set FSTAPI_DIR directly or set VERILATOR_ROOT.")
endif()

foreach(_file fstapi.h fstapi.c fastlz.c lz4.c)
  if(NOT EXISTS "${FSTAPI_DIR}/${_file}")
    message(FATAL_ERROR
      "Missing ${_file} under FSTAPI_DIR='${FSTAPI_DIR}'. "
      "Point FSTAPI_DIR at a Verilator/gtkwave include directory.")
  endif()
endforeach()

find_package(ZLIB REQUIRED)

add_executable(fst_slice_vcd
  src/main.c
  ${FSTAPI_DIR}/fstapi.c
  ${FSTAPI_DIR}/fastlz.c
  ${FSTAPI_DIR}/lz4.c
)

target_include_directories(fst_slice_vcd PRIVATE
  ${FSTAPI_DIR}
)

target_compile_definitions(fst_slice_vcd PRIVATE
  _GNU_SOURCE
)

target_compile_options(fst_slice_vcd PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_link_libraries(fst_slice_vcd PRIVATE
  ZLIB::ZLIB
)
