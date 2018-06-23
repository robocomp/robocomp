# Point Cloud Library
find_package(PCL REQUIRED)

LIST( APPEND CMAKE_MODULE_PATH $ENV{ROBOCOMP}/cmake/modules )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

MESSAGE("X X X X X X X X X X X ${PCL_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS}  ")

SET( LIBS ${LIBS} ${PCL_LIBRARY_DIRS}  ${PCL_LIBRARIES}  )
message("PCL libraries: ${LIBS}" )
