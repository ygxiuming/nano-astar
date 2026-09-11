cmake_minimum_required(VERSION 3.18...3.30)
project(nano_astar LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)

# Locate the nanobind CMake package shipped by the `nanobind` PyPI wheel.
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT
  RESULT_VARIABLE _nanobind_rc)
if(NOT _nanobind_rc EQUAL 0)
  message(FATAL_ERROR "nanobind not found for ${Python_EXECUTABLE}; "
                      "add nanobind to your build requirements")
endif()
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_core src/cpp/astar.cpp)
target_compile_features(_core PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(_core PRIVATE /utf-8 /O2)
endif()

install(TARGETS _core LIBRARY DESTINATION nano_astar)
