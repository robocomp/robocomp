
cmake_minimum_required(VERSION 2.6)
PROJECT(  client4 )

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path") 
SET(RC_COMPONENT_DEVEL_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component inary path")

MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT src/ui_${input_file}.py
      COMMAND pyside-uic src/${input_file}.ui -o src/ui_${input_file}.py
      DEPENDS src/${input_file}.ui
      COMMENT "Generating src/ui_${input_file}.py from src/${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS src/ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )

