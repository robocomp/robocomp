cmake_minimum_required(VERSION 3.10)

include( ${CMAKE_SOURCE_DIR}/cmake/robocompLocal.cmake )
include( CMakeListsSpecific.txt)

#ROBOCOMP
ROBOCOMP_INITIALIZE( ${CMAKE_SOURCE_DIR}/ )
ROBOCOMP_IDSL_TO_ICE( CommonBehavior Logger )
ROBOCOMP_ICE_TO_SRC( CommonBehavior Logger )
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Qt
include(${CMAKE_SOURCE_DIR}/cmake/modules/qt.cmake)

find_package(OpenMP REQUIRED)

# Specify build and link process
add_executable( rclogger ${SOURCES} )
target_sources( rclogger
        PRIVATE
        ${CMAKE_SOURCE_DIR}/classes/rapplication/rapplication.cpp
        ${CMAKE_SOURCE_DIR}/classes/qlog/qlog.cpp
        main.cpp
        genericmonitor.cpp
        specificmonitor.cpp
        commonbehaviorI.cpp
        genericworker.cpp
        specificworker.cpp
        loggerI.cpp
        ${SPECIFIC_SOURCES})

target_link_libraries( rclogger PRIVATE ${QT_LIBRARIES} ${Ice_LIBRARIES} PUBLIC OpenMP::OpenMP_CXX)
target_include_directories(rclogger PRIVATE
          ${CMAKE_CURRENT_BINARY_DIR}
        )
#target_compile_features(
#        ${OpenMP_C_FLAGS}
#        ${OpenMP_CXX_FLAGS}
#)
install(FILES ${EXECUTABLE_OUTPUT_PATH}/rclogger DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
