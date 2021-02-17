cmake_minimum_required( VERSION 2.6 )
project(rcmanagersimple)


MESSAGE( STATUS "Adding rule to generate the ${PROJECT_NAME} executable." )

INCLUDE( ${CMAKE_SOURCE_DIR}/cmake/robocompLocal.cmake)
robocomp_wrap_python_ui( formManagerSimple editorFormSimple )
add_custom_target(rcmanagersimple ALL
        DEPENDS ui_formManagerSimple.py ui_editorFormSimple.py )
install(PROGRAMS rcmanagersimple.py DESTINATION /opt/robocomp/bin/ RENAME rcmanagersimple )
install( FILES rcmanagerEditorSimple.py rcmanagerConfigSimple.py ${CMAKE_CURRENT_BINARY_DIR}/ui_formManagerSimple.py ${CMAKE_CURRENT_BINARY_DIR}/ui_editorFormSimple.py DESTINATION  /opt/robocomp/python/${PROJECT_NAME} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

install( DIRECTORY "share/rcmanagerSimple" DESTINATION /opt/robocomp/share )
