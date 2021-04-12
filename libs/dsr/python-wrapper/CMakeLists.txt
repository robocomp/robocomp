cmake_minimum_required( VERSION 3.10 )
project(python_wrapper
        VERSION 1.0.0
        DESCRIPTION "Robocomp dsr python wrapper library"
        LANGUAGES CXX )

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PythonLibs REQUIRED)
find_package(Python3)
find_package(pybind11 CONFIG REQUIRED)

add_library(pydsr MODULE)

target_sources(pydsr
        PRIVATE
        python_api.cpp
        )


target_link_libraries(pydsr
        PRIVATE
        Qt5::Core
        Eigen3::Eigen
        Robocomp::dsr_api
        Robocomp::dsr_core
        ${PYTHON_LIBRARIES}
        pybind11::module
        )

target_include_directories(pydsr
        PUBLIC
        # Headers of DSR Api and Core
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../api/include/>
        # Own include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        PRIVATE
        ${PYTHON_INCLUDE_DIRS}
        )


set_target_properties(pydsr PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
        Python_ADDITIONAL_VERSIONS 3.8)

target_compile_options(pydsr PRIVATE -g -fmax-errors=5 -std=c++20 )
set_target_properties(pydsr PROPERTIES PREFIX "")



# 'make install' to the correct locations (provided by GNUInstallDirs).
set(pydsr_INSTALL_PYTHONDIR "${Python3_SITEARCH}")
message(STATUS ${Python3_SITEARCH})
install(TARGETS pydsr DESTINATION ${pydsr_INSTALL_PYTHONDIR} COMPONENT module)

