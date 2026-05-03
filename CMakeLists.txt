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
    cpp_core/src/path_selector.cpp
    cpp_core/src/router.cpp
)

target_include_directories(router_core PRIVATE cpp_core/include)

option(ROUTER_ENABLE_GUROBI "Enable Gurobi-backed path selector" ON)
if(ROUTER_ENABLE_GUROBI)
    if(NOT DEFINED ENV{GUROBI_HOME})
        if(EXISTS "/opt/gurobi1301/linux64")
            set(ENV{GUROBI_HOME} "/opt/gurobi1301/linux64")
        elseif(EXISTS "/opt/gurobi1103/linux64")
            set(ENV{GUROBI_HOME} "/opt/gurobi1103/linux64")
        endif()
    endif()

    set(_gurobi_hints "")
    if(DEFINED ENV{GUROBI_HOME})
        list(APPEND _gurobi_hints "$ENV{GUROBI_HOME}")
    endif()
    list(APPEND _gurobi_hints "/opt/gurobi1301/linux64" "/opt/gurobi1103/linux64")

    unset(GUROBI_INCLUDE_DIR CACHE)
    unset(GUROBI_CPP_LIB CACHE)
    unset(GUROBI_CORE_LIB CACHE)

    find_path(GUROBI_INCLUDE_DIR
        NAMES gurobi_c++.h
        HINTS ${_gurobi_hints}
        PATH_SUFFIXES include
    )
    find_library(GUROBI_CPP_LIB
        NAMES gurobi_g++8.5
        HINTS ${_gurobi_hints}
        PATH_SUFFIXES lib
    )
    find_library(GUROBI_CORE_LIB
        NAMES gurobi130 gurobi110 gurobi130_light gurobi110_light
        HINTS ${_gurobi_hints}
        PATH_SUFFIXES lib
    )

    if(GUROBI_INCLUDE_DIR AND GUROBI_CPP_LIB AND GUROBI_CORE_LIB)
        target_compile_definitions(router_core PRIVATE ROUTER_HAS_GUROBI=1)
        target_include_directories(router_core PRIVATE ${GUROBI_INCLUDE_DIR})
        target_link_libraries(router_core PRIVATE ${GUROBI_CPP_LIB} ${GUROBI_CORE_LIB})
        get_filename_component(_gurobi_lib_dir ${GUROBI_CORE_LIB} DIRECTORY)
        set_target_properties(router_core PROPERTIES
            BUILD_RPATH "${_gurobi_lib_dir}"
            INSTALL_RPATH "${_gurobi_lib_dir}"
        )
        message(STATUS "Gurobi enabled for router_core: ${GUROBI_CORE_LIB}")
    else()
        message(STATUS "Gurobi not found; building without Gurobi path selector backend.")
    endif()
endif()
