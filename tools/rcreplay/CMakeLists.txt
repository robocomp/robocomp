cmake_minimum_required( VERSION 3.10 )
project(rcreplay
		VERSION 2021.2.1
		DESCRIPTION "RoboComp rcreplay tool"
		LANGUAGES CXX )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyside2-uic ${input_file}.ui -o ui_${input_file}.py
      DEPENDS ${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )

MESSAGE(STATUS "Adding rule to generate the monitorComp executable" )
ADD_CUSTOM_COMMAND (
	OUTPUT rcreplay
	COMMAND cp rcreplay.py rcreplay
	COMMAND chmod 755 rcreplay
)
ADD_CUSTOM_TARGET(executable ALL DEPENDS rcreplay )


# Program destination directory
SET( DEST_DIR "/opt/robocomp/bin/")
SET( ICONS_DEST_DIR "/opt/robocomp/icons/")

ROBOCOMP_WRAP_PYTHON_UI( gui frameskip )

INSTALL(FILES rcreplay DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL(FILES ui_gui.py ui_frameskip.py camera.py differentialRobot.py jointMotor.py roimant.py laser.py DESTINATION ${DEST_DIR}   PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ )
INSTALL(FILES replay-green-icon.png replay-red-icon.png DESTINATION ${ICONS_DEST_DIR}   PERMISSIONS OWNER_READ OWNER_WRITE  GROUP_READ WORLD_READ )
