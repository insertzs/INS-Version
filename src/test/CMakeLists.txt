cmake_minimum_required(VERSION 3.10)

project(INAVTests)

set(MAIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(MAIN_LIB_DIR "${MAIN_DIR}/lib")
set(MAIN_UTILS_DIR "${MAIN_DIR}/src/utils")
set(MAIN_SRC_DIR "${MAIN_DIR}/src/main")
set(TOOLCHAIN host)
set(GIT_REV test)

include("${MAIN_DIR}/cmake/main.cmake")
include("${MAIN_DIR}/cmake/settings.cmake")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/gtest.cmake)

enable_testing()
include(GoogleTest)
add_subdirectory(unit)
