###############################################################################
# Add a library target.
# name The library name.
# component The part of RoboComp that this library belongs to.
# ARGN The source files for the library.
macro(RoboComp_ADD_LIBRARY name lib_type component)
    add_library(${name} ${lib_type} ${ARGN})

    set_target_properties(${name} PROPERTIES
        VERSION ${RoboComp_VERSION}
        SOVERSION ${RoboComp_VERSION}
        DEFINE_SYMBOL "RoboCompAPI_EXPORTS")
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${name} PROPERTIES FOLDER "Libraries")
    endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT RoboComp_${component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT RoboComp_${component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT RoboComp_${component})

endmacro(RoboComp_ADD_LIBRARY)

###############################################################################
# Add a set of include files to install.
# component The part of RoboComp that the install files belong to.
# subdir The sub-directory for these include files.
# ARGN The include files.
macro(RoboComp_ADD_INCLUDES component subdir)
    install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${subdir}
        COMPONENT RoboComp_${component})
endmacro(RoboComp_ADD_INCLUDES)

##############################################################################
    # Collect subdirectories from dirname that contains filename and store them in
    #  varname.
# WARNING If extra arguments are given then they are considered as exception 
# list and varname will contain subdirectories of dirname that contains 
# fielename but doesn't belong to exception list.
# dirname IN parent directory
# filename IN file name to look for in each subdirectory of parent directory
# varname OUT list of subdirectories containing filename
# exception_list OPTIONAL and contains list of subdirectories not to account
macro(collect_library_directory_names dirname filename names dirs)
    file(GLOB globbed RELATIVE "${dirname}" "${dirname}/*/${filename}")
    if(${ARGC} GREATER 3)
        set(exclusion_list ${ARGN})
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            list(FIND exclusion_list ${dir} excluded)
            if(excluded EQUAL -1)
                set(${dirs} ${${dirs}} ${dir})
            endif(excluded EQUAL -1)
        endforeach()
    else(${ARGC} GREATER 3)
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            set(${dirs} ${${dirs}} ${dir})
        endforeach(file)      
    endif(${ARGC} GREATER 3)
    foreach(subdir ${${dirs}})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt name REGEX "set.*LIB_NAME .*\\)$")
        string(REGEX REPLACE "set.*LIB_NAME" "" name "${name}")
        string(REPLACE ")" "" name "${name}")
        string(STRIP "${name}" name)
#       message(STATUS "setting ${subdir} component name to ${name}")
        set(${names} ${${names}} ${name})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt DEPENDENCIES REGEX "set.*LIB_DEPS .*\\)")
        string(REGEX REPLACE "set.*LIB_DEPS" "" DEPENDENCIES "${DEPENDENCIES}")
        string(REPLACE ")" "" DEPENDENCIES "${DEPENDENCIES}")
        string(STRIP "${DEPENDENCIES}" DEPENDENCIES)
        string(REPLACE " " ";" DEPENDENCIES "${DEPENDENCIES}")
        if(NOT("${DEPENDENCIES}" STREQUAL ""))
            list(REMOVE_ITEM DEPENDENCIES "#")
            string(TOUPPER "RoboComp_${name}_DEPENDS" LIB_DEPENDS)
            set(${LIB_DEPENDS} ${DEPENDENCIES})
            foreach(dependee ${DEPENDENCIES})
                string(TOUPPER "RoboComp_${dependee}_DEPENDIES" LIB_DEPENDIES)
                set(${LIB_DEPENDIES} ${${LIB_DEPENDIES}} ${name})
            endforeach(dependee)
        endif(NOT("${DEPENDENCIES}" STREQUAL ""))
    endforeach(subdir)
endmacro()

###############################################################################
# Add an executable target.
# component name of the component
# interfaces the interfaces used by the component
# ARGN the source files for the library.
macro(RoboComp_ADD_COMPONENT component_name interfaces headers)
	foreach ( interface_name ${interfaces})
		#message ("interface: " ${interface_name} )
		#Do the dew: 
		message(STATUS "Adding rule to generate ${interface_name}.cpp and ${interface_name}.h from ${RoboComp_INTERFACES_DIR}/${interface_name}.ice  (slice2cpp)" )
		add_custom_command (
			OUTPUT ${interface_name}.cpp ${interface_name}.h
			COMMAND slice2cpp -I${RoboComp_INTERFACES_DIR} -I. ${RoboComp_INTERFACES_DIR}/${interface_name}.ice --output-dir .
			DEPENDS ${RoboComp_INTERFACES_DIR}/${interface_name}.ice
			COMMENT "Generating ${interface_name}.cpp and ${interface_name}.h from ${interface_name}.ice"
		)
		set(interface_files ${interface_files} ${interface_name}.cpp )
        
	endforeach (interface_name )

	INCLUDE( ${QT_USE_FILE} )
    
	include_directories( ${RoboComp_CLASSES_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${component_name}/src/ ${CMAKE_CURRENT_BINARY_DIR} )
    
	QT_WRAP_CPP( MOC_SOURCES ${headers} )
    
    add_executable( ${component_name} ${interface_files} ${ARGN} ${MOC_SOURCES})
    
	set_target_properties( ${component_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${component_name}/bin/ )
    
	target_link_libraries( ${component_name} ${QT_LIBRARIES} -lIce -lIceUtil -lIceStorm -lpthread )

	set(RoboComp_COMPONENTS ${RoboComp_COMPONENTS} ${component_name})
	install(TARGETS ${component_name} RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT RoboComp_${component_name})
        
endmacro(RoboComp_ADD_COMPONENT)

###############################################################################
# Add an executable target.
# name The executable name.
# component The part of RoboComp that this library belongs to.
# ARGN the source files for the library.
macro(RoboComp_ADD_EXECUTABLE name component)
	add_executable(${name} ${ARGN})
	target_link_libraries(${name} ${Boost_LIBRARIES} pthread m ${CLANG_LIBRARIES})
	set(RoboComp_EXECUTABLES ${RoboComp_EXECUTABLES} ${name})
	install(TARGETS ${name} RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT RoboComp_${component})
endmacro(RoboComp_ADD_EXECUTABLE)

###############################################################################
# Get the include directory name of a subsystem - return name if not set
# var Destination variable.
# name Name of the subsystem.
macro(RoboComp_GET_LIB_INCLUDE_DIR var name)
	GET_IN_MAP(${var} RoboComp_LIB_INCLUDE ${name})
	if(NOT ${var})
		set (${var} ${name})
	endif(NOT ${var})
endmacro(RoboComp_GET_LIB_INCLUDE_DIR)

###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# var The cumulative build variable. This will be set to FALSE if the
#  dependencies are not met.
# name The name of the subsystem.
# ARGN The subsystems and external libraries to depend on.
macro(RoboComp_LIB_DEPEND var name)
	set(options)
	set(oneValueArgs)
	set(multiValueArgs DEPS EXT_DEPS OPT_DEPS)
	cmake_parse_arguments(LIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
	if(LIB_DEPS)
		SET_IN_GLOBAL_MAP(RoboComp_LIB_DEPS ${name} "${LIB_DEPS}")
	endif(LIB_DEPS)
	if(LIB_EXT_DEPS)
		SET_IN_GLOBAL_MAP(RoboComp_LIB_EXT_DEPS ${name} "${LIB_EXT_DEPS}")
	endif(LIB_EXT_DEPS)
	if(LIB_OPT_DEPS)
		SET_IN_GLOBAL_MAP(RoboComp_LIB_OPT_DEPS ${name} "${LIB_OPT_DEPS}")
	endif(LIB_OPT_DEPS)
	foreach(dep ${LIB_DEPS})
			RoboComp_GET_LIB_INCLUDE_DIR(include_dir ${dep})
			include_directories(${RoboComp_LIBS_DIR}/${include_dir}/include)
	endforeach(dep)
endmacro(RoboComp_LIB_DEPEND)
