
# Add a library target from robocomp
# _name The library name.
# ARGN The source files for the library.
macro(RoboComp_ADD_LIBRARY name)
    add_library(${_name} ${PCL_LIB_TYPE} ${ARGN})
    # must add links
    target_link_libraries(${name} )
    
    set_target_properties(${name} PROPERTIES
        VERSION ${RoboComp_VERSION})
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${name} PROPERTIES FOLDER "Classes")
    endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${name}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${name}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${name})

endmacro(RoboComp_ADD_LIBRARY)