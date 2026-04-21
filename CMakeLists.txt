cmake_minimum_required(VERSION 3.18)
project(interactive_router_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYBIND11_FINDPYTHON ON)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(router_core
    cpp_core/pybind_api.cpp
    cpp_core/src/grid_3d.cpp
    cpp_core/src/net_analysis.cpp
    cpp_core/src/router.cpp
)

target_include_directories(router_core PRIVATE cpp_core/include)
