cmake_minimum_required( VERSION 2.6 )

MACRO( ROBOCOMP_WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    MESSAGE(STATUS "Adding rule to generate ui_${input_file}.py from ${input_file}.ui" )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyuic4 ${CMAKE_CURRENT_SOURCE_DIR}/${input_file}.ui -o ui_${input_file}.py
      DEPENDS        ${CMAKE_CURRENT_SOURCE_DIR}/${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )

MESSAGE(STATUS "Adding rule to generate the rcmanager executable" )



# Program destination directory
SET( DEST_DIR "/opt/robocomp/bin/")

ROBOCOMP_WRAP_PYTHON_UI( formManager editorForm )
ADD_EXECUTABLE( rcmanager rcmanager.c )
ADD_CUSTOM_TARGET(exec_rcmanager ALL DEPENDS rcmanager )

INSTALL(FILES rcmanager DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL(FILES rcmanager.py rcmanagerEditor.py rcmanagerConfig.py ui_formManager.py ui_editorForm.py DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ  WORLD_EXECUTE )

INSTALL(DIRECTORY "share/rcmanager" DESTINATION /opt/robocomp/share )



