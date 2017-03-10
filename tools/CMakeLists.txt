cmake_minimum_required( VERSION 2.6 )

find_package(Qt4)

include( rcinnerModelSimulator/tool_config.cmake )

include_directories ( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories ( ${CMAKE_CURRENT_SOURCE_DIR}/include )

RoboComp_ADD_COMPONENT( rcis "${rcis_interfaces}" "${rcis_headers}" ${rcis_src} )
target_link_libraries( rcis robocomp_innermodel )
