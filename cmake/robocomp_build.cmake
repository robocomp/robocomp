###############################################################################
# Set the destination directories for installing stuff.
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
# subdirectory named after the library in question (e.g.
# "registration/blorgle.h")


macro(ROBOCOMP_INITIALIZE)
    # Set root directory
    SET( ROBOCOMP_ROOT ${ARGN} )
    MESSAGE(STATUS "BU RoboComp root is now set to ${ROBOCOMP_ROOT}")
    set(ROBOCOMP_INSTALL_DIR
        "${ROBOCOMP_ROOT}")
    set(ROBOCOMP_LIB_INSTALL_DIR "${ROBOCOMP_INSTALL_DIR}/lib")
    set(ROBOCOMP_INCLUDE_INSTALL_DIR "${ROBOCOMP_INSTALL_DIR}/include/")
    set(ROBOCOMP_DOC_INSTALL_DIR "${ROBOCOMP_INSTALL_DIR}/doc")
    set(ROBOCOMP_BIN_INSTALL_DIR "${ROBOCOMP_INSTALL_DIR}/bin")
    set(CROBOCOMP_ONFIG_INSTALL_DIR "${ROBOCOMP_INSTALL_DIR}/etc-default")
    set(PROBOCOMP_KGCFG_INSTALL_DIR "${ROBOCOMP_LIB_INSTALL_DIR}/pkgconfig")
    include(${ROBOCOMP_ROOT}/libs)
endmacro(ROBOCOMP_INITIALIZE)


function(search_idsl RETURN_IDSL_PATH IFACE_NAME SLICE_PATHS )
    set(found FALSE)
    set(SLICE_PATHS ${SLICE_PATHS} ${ARGN})
    FOREACH (SPATH ${SLICE_PATHS})
        IF (EXISTS "${SPATH}/${IFACE_NAME}.idsl")
            set("${RETURN_IDSL_PATH}" "${SPATH}" PARENT_SCOPE)
            set(found TRUE)
            break()
        ENDIF (EXISTS "${SPATH}/${IFACE_NAME}.idsl")
    ENDFOREACH (SPATH ${SLICE_PATHS})
    if (found EQUAL FALSE)
        MESSAGE(FATAL_ERROR "${IFACE_NAME}.idsl not found in (${SLICE_PATH}).")
    endif(found EQUAL FALSE)
endfunction(search_idsl)

function(idsl_to_iceE RETURN_ICE_PATH IDSL_PATH IFACE_NAME )
    MESSAGE(STATUS "BU Adding_ rule to generate ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.ice from ${IDSL_PATH}/${IFACE_NAME}.idsl")
    STRING (REPLACE "/" "_" SPECIFIC_TARGET "${CMAKE_CURRENT_SOURCE_DIR}")
    add_custom_command(
            COMMAND "${CMAKE_HOME_DIRECTORY}/tools/robocompdsl/robocompdsl.py" ${IDSL_PATH}/${IFACE_NAME}.idsl ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.ice
            DEPENDS ${IDSL_PATH}/${IFACE_NAME}.idsl
            COMMENT "BU robocompdsl ${IDSL_PATH}/${IFACE_NAME}.idsl ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.ice"
            TARGET ICES_${SPECIFIC_TARGET}
    )
    set("${RETURN_ICE_PATH}" ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
endfunction(idsl_to_iceE)

function(ice_to_src RETURN_SOURCES ICE_PATH IFACE_NAME)
    # Making the new .ice depend on the PREVIOUS_ICE force it to keep the creation order
    MESSAGE(STATUS "BU ice=>h/cpp: Adding rule to generate ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.[h-cpp] from ${ICE_PATH}/${IFACE_NAME}.ice")
    STRING (REPLACE "/" "_" SPECIFIC_TARGET "${CMAKE_CURRENT_SOURCE_DIR}")
    add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.cpp
            COMMAND slice2cpp ${ICE_PATH}/${IFACE_NAME}.ice -I${ICE_PATH}/ --output-dir ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ICES_${SPECIFIC_TARGET}
            COMMENT "BU ice=>h/cpp: Generating ${IFACE_NAME}.h and ${IFACE_NAME}.cpp from ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.ice"
    )

    SET (${RETURN_SOURCES} ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.cpp PARENT_SCOPE)
    SET_PROPERTY(SOURCE ${CMAKE_CURRENT_BINARY_DIR}/${IFACE_NAME}.cpp PROPERTY SKIP_AUTOGEN ON)
endfunction(ice_to_src)

function(robocomp_idsl_to_src IDSL_SRCS)
    message(STATUS "Converting idsls to src: ${ARGN}")
    STRING (REPLACE "/" "_" SPECIFIC_TARGET "${CMAKE_CURRENT_SOURCE_DIR}")
    ADD_CUSTOM_TARGET(ICES_${SPECIFIC_TARGET} ALL)
    SET( SLICE_PATHS "$ENV{SLICE_PATH};$ENV{ROBOCOMP}/interfaces/IDSLs;/opt/robocomp/interfaces/IDSLs;./ice_files/;/home/robolab/robocomp/interfaces/IDSLs;")
    message(STATUS "Search paths: ${SLICE_PATHS}")
    SET(NEW_SRCS "")
    FOREACH( IFACE_NAME ${ARGN} )
        search_idsl(IDSL_PATH ${IFACE_NAME} "${SLICE_PATHS}")
        message(STATUS "FOUND in ${IDSL_PATH}")
        idsl_to_iceE(ICE_PATH ${IDSL_PATH} ${IFACE_NAME} )
        message(STATUS "ICE path will be ${ICE_PATH}")
        ice_to_src(SRC_FILES "${ICE_PATH}" "${IFACE_NAME}")
        set(NEW_SRCS ${NEW_SRCS} ${SRC_FILES})
    ENDFOREACH( IFACE_NAME )
    set(${IDSL_SRCS} ${NEW_SRCS} PARENT_SCOPE)
endfunction(robocomp_idsl_to_src)