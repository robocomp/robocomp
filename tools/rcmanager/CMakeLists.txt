cmake_minimum_required( VERSION 2.6 )

MACRO( ROBOCOMP_WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    MESSAGE(STATUS "Adding rule to generate ui_${input_file}.py from ${input_file}.ui" )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyuic4 ${input_file}.ui -o ui_${input_file}.py
      DEPENDS ${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )

MESSAGE(STATUS "Adding rule to generate the rcmanager executable" )
ADD_CUSTOM_COMMAND (
	OUTPUT rcmanager
	COMMAND gcc rcmanager.c -o rcmanager
	COMMAND chmod 755 rcmanager
)
ADD_CUSTOM_TARGET(exec_rcmanager ALL DEPENDS rcmanager )


# Program destination directory
SET( DEST_DIR "/opt/robocomp/bin/")

ROBOCOMP_WRAP_PYTHON_UI( formManager editorForm )
INSTALL(FILES rcmanager DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL(FILES rcmanager.py rcmanagerEditor.py rcmanagerConfig.py ui_formManager.py ui_editorForm.py DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ  WORLD_EXECUTE )
INSTALL(FILES share/drawing_left.png share/drawing_right.png share/drawing_red.png share/drawing_green.png share/up.png share/down.png share/pixmap.png  DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE WORLD_READ )


