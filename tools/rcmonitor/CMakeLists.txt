#cmake_minimum_required( VERSION 2.8 )

MACRO( ROBOCOMP_WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    MESSAGE(STATUS "Adding rule to generate ui_${input_file}.py from ${input_file}.ui" )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyuic4 ${CMAKE_SOURCE_DIR}/${input_file}.ui -o ui_${input_file}.py
      DEPENDS ${CMAKE_SOURCE_DIR}/${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )

MESSAGE(STATUS "Adding rule to generate the RCMonitor executable" )
ADD_CUSTOM_COMMAND (
	OUTPUT rcmonitor
	COMMAND cp ${CMAKE_SOURCE_DIR}/rcmonitor.py rcmonitor
	COMMAND chmod 755 rcmonitor
)
ADD_CUSTOM_TARGET(exec_monitorcomp ALL DEPENDS rcmonitor )


# Program destination directory
SET( DEST_DIR "/opt/robocomp/bin/")

ROBOCOMP_WRAP_PYTHON_UI( formMonitor openA period jointmotorDlg headntpDlg headt2pDlg forkDlg floorDlg armRocioDlg kinectDlg aprendizDlg smarTestDlg depthBusDlg)
INSTALL(FILES rcmonitor       DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL(FILES ui_formMonitor.py DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
INSTALL(FILES ui_openA.py       DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
INSTALL(FILES ui_period.py      DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )



