
cmake_minimum_required(VERSION 2.6)
PROJECT( agmagentsim )

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path") 
SET(RC_COMPONENT_DEVEL_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component binary path")

MESSAGE( "SRC ${CMAKE_CURRENT_SOURCE_DIR}")
MESSAGE( "BIN ${CMAKE_CURRENT_BINARY_DIR}")


MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    set (uiif ${CMAKE_CURRENT_SOURCE_DIR}/src/${input_file}.ui )
    set (uiof ${CMAKE_CURRENT_BINARY_DIR}/ui_${input_file}.py )
  
    ADD_CUSTOM_COMMAND (
      OUTPUT ${uiof}
      COMMAND pyside-uic ${uiif} -o ${uiof}
      DEPENDS ${uiif}
      COMMENT "Generating ${uiof} from ${uiif}"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ${uiof} )
	INSTALL(PROGRAMS ${uiof} DESTINATION ${INST_BIN_PATH}/agmagentsim_py/ )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )

WRAP_PYTHON_UI( agmagentsimUI )


INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/src/agmagentsim.py" DESTINATION ${INST_BIN_PATH}/ RENAME agmagentsim )
INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/src/agmcommonbehaviorI.py" DESTINATION ${INST_BIN_PATH}/agmagentsim_py/ )
INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/src/agmexecutivetopicI.py" DESTINATION ${INST_BIN_PATH}/agmagentsim_py/ )
INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/src/genericworker.py" DESTINATION ${INST_BIN_PATH}/agmagentsim_py/ )
INSTALL(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/src/specificworker.py" DESTINATION ${INST_BIN_PATH}/agmagentsim_py/ )


          


