cmake_minimum_required( VERSION 3.10 )
project(rcmonitor
        VERSION 2021.2.1
        DESCRIPTION "RoboComp RCMonitor tool"
        LANGUAGES CXX )

MACRO( ROBOCOMP_WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    MESSAGE(STATUS "Adding rule to generate ui_${input_file}.py from ${input_file}.ui" )
    ADD_CUSTOM_COMMAND (
      OUTPUT ui_${input_file}.py
      COMMAND pyside2-uic ${CMAKE_CURRENT_SOURCE_DIR}/${input_file}.ui -o ui_${input_file}.py
      DEPENDS        ${CMAKE_CURRENT_SOURCE_DIR}/${input_file}.ui
      COMMENT "Generating ui_${input_file}.py from ${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_PYTHON_UI )

# Program destination directory
SET( DEST_DIR "/opt/robocomp/bin/")

ROBOCOMP_WRAP_PYTHON_UI( formMonitor openA period jointmotorDlg headntpDlg headt2pDlg forkDlg floorDlg kinectDlg rgbdbusDlg depthBusDlg)
INSTALL(FILES rcmonitor.py       DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE RENAME rcmonitor)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/ui_formMonitor.py DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/ui_openA.py       DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/ui_period.py      DESTINATION ${DEST_DIR} PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
