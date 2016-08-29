cmake_minimum_required( VERSION 2.8)

#  Compiler options  #
#####################
set(CMAKE_CXX_FLAGS "-Wall -Wfatal-errors " )
INCLUDE_DIRECTORIES ( . libagm )


#  Optional features and options  #
###################################
option (PYTHON_BINDINGS "Build python bindings" OFF)
option (ROBOCOMP_SUPPORT "Build with support for RoboComp" OFF)

#  Set sources  #
#################
SET (AGM_CODE
  libagm/agm_misc_functions.cpp
  libagm/agm_model.cpp
  libagm/agm_modelEdge.cpp
  libagm/agm_modelPrinter.cpp
  libagm/agm_modelSymbols.cpp
  libagm/agm_modelConverter.cpp
  libagm/agm_plan.cpp
  libagm/agm_inner.cpp
  libagm/graphModelViewer.cpp
)


SET (HEADER_FILES
  ${CMAKE_SOURCE_DIR}/libagm/agm.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelDrawer.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelEdge.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelException.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_model.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelPrinter.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelSymbols.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_misc_functions.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_modelConverter.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_config.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_plan.h
  ${CMAKE_SOURCE_DIR}/libagm/agm_inner.h
  ${CMAKE_SOURCE_DIR}/libagm/graphModelViewer.h
)
#  Show enabled optional features  #
####################################
MESSAGE(STATUS "\n\nOptional features:")

if (PYTHON_BINDINGS)
	MESSAGE(STATUS "PYTHON BINDINGS:        YES")
else (PYTHON_BINDINGS)
	MESSAGE(STATUS "PYTHON BINDINGS:        NO")
endif (PYTHON_BINDINGS)

if (ROBOCOMP_SUPPORT)
	MESSAGE(STATUS "ROBOCOMP SUPPORT:       YES")
	SET(ROBOCOMP_SUPPORT_VAL "1")
else (ROBOCOMP_SUPPORT)
	MESSAGE(STATUS "ROBOCOMP SUPPORT:       NO")
	SET(ROBOCOMP_SUPPORT_VAL "0")
endif (ROBOCOMP_SUPPORT)

MESSAGE(STATUS "\n\n")


#  Handle optional features  #
##############################

# Python bindings
if (PYTHON_BINDINGS)
	# Add bindings source code file
	SET(SOURCES ${SOURCES} libagm/agm_python.cpp)
endif (PYTHON_BINDINGS)

#  Boost library  #
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS)
	#find_package(Boost COMPONENTS system thread python REQUIRED)
	SET(Boost_LIBRARIES -lboost_system -lboost_thread -lboost_python )
else (PYTHON_BINDINGS)
	SET(Boost_LIBRARIES -lboost_system -lboost_thread )
	#find_package(Boost COMPONENTS system thread REQUIRED)
endif (PYTHON_BINDINGS)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
# (libpython)
if (PYTHON_BINDINGS)
	find_package(PythonLibs REQUIRED)
	MESSAGE(STATUS "PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS}")
	MESSAGE(STATUS "PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES}")
	include_directories(${PYTHON_INCLUDE_DIRS})
	link_directories(${PYTHON_LIBRARIES})
endif (PYTHON_BINDINGS)

# RoboComp support
if (ROBOCOMP_SUPPORT)
	MESSAGE(STATUS "ROBOCOMP PATH:        $ENV{ROBOCOMP}")
	INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
	ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
	SET(ICE_LIBS -lIce)
	ROBOCOMP_WRAP_ICE( AGMWorldModel AGMExecutive Planning AGMCommonBehavior )
	ADD_DEFINITIONS(-DROBOCOMP_SUPPORT=1)
	SUBDIRS (${CMAKE_SOURCE_DIR}/tools/agmmission tools/agminner)
endif (ROBOCOMP_SUPPORT)

# LibXML
FIND_PACKAGE(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
set(LIBS ${LIBS} ${LIBXML2_LIBRARIES} )

# Qt4
ADD_DEFINITIONS( -Wall  -DQT_XML_LIB -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( CMAKE_AUTOMOC TRUE )
INCLUDE( ${QT_USE_FILE} )
# QT4_WRAP_CPP(XMLVIEWER3D_MOCS libagm/graphModelViewer.h )

# OSG
FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgViewer)
FIND_PACKAGE(osgText)
FIND_PACKAGE(osgQt)
SET(OSG_LIBS ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGQT_LIBRARY})
INCLUDE_DIRECTORIES(${OPENTHREADS_INCLUDE_DIR} ${OSG_INCLUDE_DIR})
LINK_DIRECTORIES(${OSG_LIB_DIR})


#  Macro for compiling UI files and wrap command  #
###################################################
MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT                                                              ${CMAKE_BINARY_DIR}/ui_${input_file}.py
      COMMAND pyside-uic ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.ui -o ${CMAKE_BINARY_DIR}/ui_${input_file}.py
      DEPENDS            ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.ui
      COMMENT "Generating ${CMAKE_BINARY_DIR}/ui_${input_file}.py from ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ${CMAKE_BINARY_DIR}/ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )
MACRO( WRAP_PYTHON_QRC )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT                                                               ${CMAKE_BINARY_DIR}/${input_file}_rc.py
      COMMAND pyside-rcc ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.qrc -o ${CMAKE_BINARY_DIR}/${input_file}_rc.py
      DEPENDS            ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.qrc
      COMMENT "Generating ${CMAKE_BINARY_DIR}/${input_file}_rc.py from ${CMAKE_SOURCE_DIR}/tools/ui/${input_file}.qrc"
    )
    ADD_CUSTOM_TARGET(${input_file}_rc ALL DEPENDS ${CMAKE_BINARY_DIR}/${input_file}_rc.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_QRC )
WRAP_PYTHON_UI( guiAGGLEditor guiGraphViewer appearance )
WRAP_PYTHON_QRC( resource )


#  Library building  #
######################
if (ROBOCOMP_SUPPORT)
   add_library(agmdummy SHARED AGMWorldModel.cpp AGMExecutive.cpp AGMCommonBehavior.cpp Planning.cpp )
   set(ROBOCOMP_ADDITIONAL_SRC AGMExecutive.cpp Planning.cpp )
endif (ROBOCOMP_SUPPORT)

add_library(agm SHARED ${AGM_CODE} ${XMLVIEWER3D_MOCS} )
target_link_libraries(agm ${Boost_LIBRARIES} ${LIBS} ${PYTHON_LIBRARIES} ${OSG_LIBS} ${QT_LIBRARIES} )
add_dependencies(agm agmdummy)

add_executable ( testagm tools/testagm.cpp ${ROBOCOMP_ADDITIONAL_SRC} )
target_link_libraries ( testagm agm -L. -L.. -lagm ${LIBS} ${ICE_LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} )
add_dependencies(testagm agm)

add_executable ( agm_xml2pddlProblem tools/agm_xml2pddlProblem.cpp ${ROBOCOMP_ADDITIONAL_SRC} )
target_link_libraries ( agm_xml2pddlProblem agm -L. -L.. -lagm ${LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} )
add_dependencies(testagm agm)

add_executable ( agm_xmlViewer3D ${CMAKE_SOURCE_DIR}/tools/agm_xmlViewer3D.cpp ${ROBOCOMP_ADDITIONAL_SRC} )
target_link_libraries ( agm_xmlViewer3D agm -L. -L.. -lagm ${LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} )
add_dependencies(testagm agm)



#  Doxygen  #
#############
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_BINARY_DIR}/doc/Doxyfile @ONLY)
  add_custom_target(doc bash ${CMAKE_SOURCE_DIR}/doc/makedoc.sh ${CMAKE_SOURCE_DIR} ${DOXYGEN_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
  MESSAGE( STATUS "Doxygen found.")
else(DOXYGEN_FOUND)
  MESSAGE( WARNING "Doxygen not found. You will not be able to generate the documentation.")
endif(DOXYGEN_FOUND)


# Configure actual header with the proper options
configure_file ("${CMAKE_SOURCE_DIR}/libagm/agm_config.in.h" "${CMAKE_SOURCE_DIR}/libagm/agm_config.h")



#  Installation  #
##################
SET( INST_BIN_PATH   ${CMAKE_INSTALL_PREFIX}/bin )
SET( INST_LIB_PATH   ${CMAKE_INSTALL_PREFIX}/lib )
SET( INST_INCL_PATH  ${CMAKE_INSTALL_PREFIX}/include )
SET( INST_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/agm )

INSTALL(FILES ${CMAKE_BINARY_DIR}/libagm.so DESTINATION ${INST_LIB_PATH}/ )
INSTALL(FILES ${HEADER_FILES} DESTINATION ${INST_INCL_PATH}/ )
INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/agm_xml2pddlProblem DESTINATION ${INST_BIN_PATH}/ )
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/tools/agm_xmlViewer.py DESTINATION ${INST_BIN_PATH}/ RENAME agm_xmlViewer )
INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/agm_xmlViewer3D DESTINATION ${INST_BIN_PATH}/ RENAME agm_xmlViewer3D )
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/tools/AGGLEditor.py DESTINATION ${INST_BIN_PATH}/ RENAME aggleditor)
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/tools/aggltypes.py DESTINATION ${INST_BIN_PATH}/ RENAME aggltypes)
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/tools/aggl2pddl DESTINATION ${INST_BIN_PATH}/ )
INSTALL(PROGRAMS ${CMAKE_SOURCE_DIR}/tools/aggl2agglpy DESTINATION ${INST_BIN_PATH}/ )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplanner.py" DESTINATION ${INST_BIN_PATH}/ RENAME agglplanner )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplanner.py" DESTINATION ${INST_SHARE_PATH}/ RENAME agglplanner.py )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplanchecker.py" DESTINATION ${INST_BIN_PATH}/ RENAME agglplanchecker )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplanchecker.py" DESTINATION ${INST_SHARE_PATH}/ RENAME agglplanchecker.py )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplan.py" DESTINATION ${INST_BIN_PATH}/ RENAME agglplan )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglplan.py" DESTINATION ${INST_SHARE_PATH}/ RENAME agglplan.py )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/AGGLPlanner/agglcheckhierarchical.py" DESTINATION ${INST_BIN_PATH}/ RENAME agglcheckhierarchical )
INSTALL(PROGRAMS "${CMAKE_SOURCE_DIR}/python/pyparsinglocal.py" DESTINATION ${INST_SHARE_PATH}/ RENAME pyparsinglocal.py )

if (ROBOCOMP_SUPPORT)
	INSTALL(FILES ${CMAKE_BINARY_DIR}/AGMCommonBehavior.h DESTINATION ${INST_INCL_PATH}/ )
	INSTALL(FILES ${CMAKE_BINARY_DIR}/AGMWorldModel.h DESTINATION ${INST_INCL_PATH}/ )
	INSTALL(FILES ${CMAKE_BINARY_DIR}/Planning.h DESTINATION ${INST_INCL_PATH}/ )
	INSTALL(FILES ${CMAKE_BINARY_DIR}/AGMExecutive.h DESTINATION ${INST_INCL_PATH}/ )
endif (ROBOCOMP_SUPPORT)

INSTALL(PROGRAMS python/parseAGGL.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/parseQuantifiers.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/pddlAGGL.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/xmlModelParser.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/generateAGGLPlannerCode.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/AGMModule.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/AGMModelConversion.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS python/AGGL.py DESTINATION ${INST_SHARE_PATH})

INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/ui_guiAGGLEditor.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/ui_guiGraphViewer.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/ui_appearance.py DESTINATION ${INST_SHARE_PATH})
INSTALL(PROGRAMS ${CMAKE_BINARY_DIR}/resource_rc.py DESTINATION ${INST_SHARE_PATH})

INSTALL(PROGRAMS tools/aggl_completion.bash DESTINATION /etc/bash_completion.d/ RENAME AGGLEditor)


ADD_DEFINITIONS( -std=c++11 )

