

include( RCInnerModelSimulator/tool_config.cmake )

include_directories ( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories ( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories(
    /usr/lib/x86_64-linux-gnu/c++11
)

RoboComp_ADD_COMPONENT( rcis "${rcis_interfaces}" "${rcis_headers}" ${rcis_src} )
target_link_libraries( rcis robocomp_innermodel )