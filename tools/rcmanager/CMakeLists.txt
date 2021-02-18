cmake_minimum_required( VERSION 3.10 )
project(rcmanager
        VERSION 2021.2.1
        DESCRIPTION "RoboComp RCManager tool"
        LANGUAGES CXX )


MESSAGE( STATUS "Adding rule to generate the ${PROJECT_NAME} executable." )

INCLUDE( ${CMAKE_SOURCE_DIR}/cmake/robocompLocal.cmake)
robocomp_wrap_python_ui( formManager editorForm )
add_custom_target(rcmanager ALL
        DEPENDS ui_formManager.py ui_editorForm.py )
install(PROGRAMS main.py DESTINATION /opt/robocomp/bin/ RENAME rcmanager )
install( FILES controller.py logger.py model.py rcmanagerSignals.py viewer.py xmlreader.py yakuake_support.py ${CMAKE_CURRENT_BINARY_DIR}/ui_formManager.py ${CMAKE_CURRENT_BINARY_DIR}/ui_editorForm.py DESTINATION  /opt/robocomp/python/${PROJECT_NAME} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

install( DIRECTORY "share/rcmanager" DESTINATION /opt/robocomp/share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        )

install( DIRECTORY "widgets/" DESTINATION /opt/robocomp/python/${PROJECT_NAME}/widgets
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        )
