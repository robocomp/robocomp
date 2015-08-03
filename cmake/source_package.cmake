
find_program(DEBUILD_EXECUTABLE debuild)
find_program(DPUT_EXECUTABLE dput)

if(NOT DEBUILD_EXECUTABLE OR NOT DPUT_EXECUTABLE)
  MESSAGE(WARNING "could'nt find debuild or dput" )
  return()
endif(NOT DEBUILD_EXECUTABLE OR NOT DPUT_EXECUTABLE)

IF(NOT CPACK_DEBIAN_PACKAGE_NAME)
  STRING(TOLOWER "${CPACK_PACKAGE_NAME}" CPACK_DEBIAN_PACKAGE_NAME)
ENDIF(NOT CPACK_DEBIAN_PACKAGE_NAME)
MESSAGE(STATUS "Debian package name: " ${CPACK_DEBIAN_PACKAGE_NAME})

######Read description file
MESSAGE(STATUS "reading description file: " ${CPACK_PACKAGE_DESCRIPTION_FILE})
file(STRINGS ${CPACK_PACKAGE_DESCRIPTION_FILE} DESC_LINES)
foreach(LINE ${DESC_LINES})
  set(DEB_LONG_DESCRIPTION "${DEB_LONG_DESCRIPTION} ${LINE}\n")
endforeach(LINE ${DESC_LINES})

######clean Debian directory
file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/Debian")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/Debian")
set(DEBIAN_SOURCE_ORIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/Debian/${CPACK_DEBIAN_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
MESSAGE(STATUS "Debian package source-orig-dir: " ${DEBIAN_SOURCE_ORIG_DIR})

#######copy of source
if( CPACK_DEBIAN_PACKAGE_SOURCE_COPY )
  execute_process(COMMAND ${CPACK_DEBIAN_PACKAGE_SOURCE_COPY} "${CMAKE_SOURCE_DIR}" "${DEBIAN_SOURCE_ORIG_DIR}")
else( CPACK_DEBIAN_PACKAGE_SOURCE_COPY )
  MESSAGE(STATUS "Copying files from ${DEB_SRC_DIR} to ${DEBIAN_SOURCE_ORIG_DIR}")
  file(COPY "${DEB_SRC_DIR}/" DESTINATION "${DEBIAN_SOURCE_ORIG_DIR}" REGEX "(build|debian|components)$" EXCLUDE)
  MESSAGE(STATUS "Removing .git from source-orig-dir. ")
  execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory "${DEBIAN_SOURCE_ORIG_DIR}/.git")
endif( CPACK_DEBIAN_PACKAGE_SOURCE_COPY )

######create the original source tar
 MESSAGE(STATUS " creating source tar " )
 if($(CMAKE_VERSION) VERSION_GREATER 3.0.0)
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar czf "${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}.orig.tar.gz" "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}" 
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Debian)
 else()
 execute_process(COMMAND tar czf "${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}.orig.tar.gz" "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}" 
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Debian)
 endif()
 MESSAGE(STATUS " tar created." )
 file(COPY Debian/${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}.orig.tar.gz DESTINATION Debian/tmp )

########start creating source packages for all releases
foreach(RELEASE ${CPACK_DEBIAN_DISTRIBUTION_RELEASES})
  MESSAGE(STATUS " processing release: " ${RELEASE})
  file(COPY Debian/tmp/${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}.orig.tar.gz DESTINATION Debian )
  file(RENAME Debian/${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}.orig.tar.gz  "Debian/${CPACK_DEBIAN_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_DISTRIBUTION_NAME}1~${RELEASE}1.orig.tar.gz" )

  ##############################################################################
  #create the debian directory
  set(DEBIAN_SOURCE_DIR "${DEBIAN_SOURCE_ORIG_DIR}-${CPACK_DEBIAN_DISTRIBUTION_NAME}1~${RELEASE}1")
  MESSAGE(STATUS "  release source dir: " ${DEBIAN_SOURCE_DIR})
  set(RELEASE_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_DISTRIBUTION_NAME}1~${RELEASE}1")
  MESSAGE(STATUS "  release package version: " ${RELEASE_PACKAGE_VERSION})
  file(MAKE_DIRECTORY ${DEBIAN_SOURCE_DIR}/debian)

   ##############################################################################
   # debian/copyright
    MESSAGE(STATUS "  creating debian/coyright.")
    set(DEBIAN_COPYRIGHT ${DEBIAN_SOURCE_DIR}/debian/copyright)
    execute_process(COMMAND ${CMAKE_COMMAND} -E
      copy ${CPACK_RESOURCE_FILE_LICENSE} ${DEBIAN_COPYRIGHT}
      )
   ##############################################################################
   # debian/compat
   file(WRITE ${DEBIAN_SOURCE_DIR}/debian/compat "9")

   ##############################################################################
   # debian/source/format
   file(WRITE ${DEBIAN_SOURCE_DIR}/debian/source/format "3.0 (quilt)")

   ##############################################################################
   #extra control files
   MESSAGE(STATUS  "copying postinst and preinst and postrm")
    file(COPY ${DEB_SRC_DIR}/debian/postinst ${DEB_SRC_DIR}/debian/preinst ${DEB_SRC_DIR}/debian/postrm
          DESTINATION ${DEBIAN_SOURCE_DIR}/debian
         )

   ##############################################################################
   #copying rule file
   MESSAGE(STATUS  "copying rules")
    file(COPY ${DEB_SRC_DIR}/debian/rules DESTINATION ${DEBIAN_SOURCE_DIR}/debian)

  ##############################################################################
   #copying fix_up file
   MESSAGE(STATUS  "copying ")
    file(COPY ${DEB_SRC_DIR}/debian/fixup_deb.sh.in DESTINATION ${DEBIAN_SOURCE_DIR}/debian)

  ##############################################################################
   #copying readme file
   MESSAGE(STATUS  "copying readme")
    file(COPY ${DEB_SRC_DIR}/debian/README.Debian DESTINATION ${DEBIAN_SOURCE_DIR}/debian)

  ##############################################################################
  # debian/control

  MESSAGE(STATUS "  creating debian/control file.")
  set(DEBIAN_CONTROL ${DEBIAN_SOURCE_DIR}/debian/control)
    file(WRITE ${DEBIAN_CONTROL}
      "Source: ${CPACK_DEBIAN_PACKAGE_NAME}\n"
      "Section: ${CPACK_DEBIAN_PACKAGE_SECTION}\n"
      "Priority: ${CPACK_DEBIAN_PACKAGE_PRIORITY}\n"
      "Maintainer: ${CPACK_PACKAGE_CONTACT}\n"
      "Build-Depends: "
      )

  MESSAGE(STATUS "   build-depency: " ${DEBSRC_BUILD_DEPENDS})
    foreach(DEP ${DEBSRC_BUILD_DEPENDS})
      MESSAGE(STATUS "   build-depency: " ${DEP})
      file(APPEND ${DEBIAN_CONTROL} "${DEP}, ")
    endforeach(DEP ${DEBSRC_BUILD_DEPENDS})

    file(APPEND ${DEBIAN_CONTROL} "\n"
      "Standards-Version: 3.9.6\n"
      "Homepage: ${CPACK_PACKAGE_VENDOR}\n"
      "Vcs-Git:https://github.com/robocomp/robocomp.git\n"
      "\n"
      "Package: ${CPACK_DEBIAN_PACKAGE_NAME}\n"
      "Architecture: any\n"
      "Depends: "
      )

  set(DEBHELP_DEPENDS "\${misc:Depends}")
  file(APPEND ${DEBIAN_CONTROL} "${DEBHELP_DEPENDS}, ")

  foreach(DEP ${DEBSRC_PACKAGE_DEPENDS})
      MESSAGE(STATUS "   package-depency: " ${DEP})
      file(APPEND ${DEBIAN_CONTROL} "${DEP}, ")
  endforeach(DEP ${DEBSRC_PACKAGE_DEPENDS})  

  file(APPEND ${DEBIAN_CONTROL} "\n"
      "Description: ${CPACK_PACKAGE_DISPLAY_NAME} ${CPACK_PACKAGE_DESCRIPTION_SUMMARY}\n"
      "${DEB_LONG_DESCRIPTION}"
    )

    MESSAGE(STATUS "  creating debian/control DONE.")
   
    ##############################################################################
    # debian/changelog
    set(DEBIAN_CHANGELOG ${DEBIAN_SOURCE_DIR}/debian/changelog)
    execute_process(COMMAND date -R  OUTPUT_VARIABLE DATE_TIME)
    file(STRINGS "${DEB_SRC_DIR}/debian/changelog" lineOne REGEX "urgency=low")
    string(REGEX MATCH "-0ppa[0-9]+" var ${lineOne})
    string(SUBSTRING ${var} 5 -1 PPA_NUMBER)
    
    message(STATUS "num ${var}")
    if(${DEB_SOURCE_CHANGES} MATCHES "^CHANGED$")
      set(PPA_NUMBER 1)
    else()
      MATH(EXPR PPA_NUMBER "${PPA_NUMBER}+1")
    endif()
    message(STATUS "ppa num ${PPA_NUMBER}")
    file(WRITE ${DEBIAN_CHANGELOG}
      "${CPACK_DEBIAN_PACKAGE_NAME} (${RELEASE_PACKAGE_VERSION}-0ppa${PPA_NUMBER}) ${RELEASE}; urgency=low\n\n"
      "  * Package built with CMake\n\n"
      " -- ${CPACK_PACKAGE_CONTACT}  ${DATE_TIME}"
      )
    #update src changelog
    file(COPY ${DEBIAN_SOURCE_DIR}/debian/changelog DESTINATION ${DEB_SRC_DIR}/debian)

    ##############################################################################
    # debuild -S
    if(${DEB_SOURCE_CHANGES} MATCHES CHANGED)
      set(DEBUILD_OPTIONS "-sa")
    else()
      set(DEBUILD_OPTIONS "-sd")
    endif()
    set(SOURCE_CHANGES_FILE "${CPACK_DEBIAN_PACKAGE_NAME}_${RELEASE_PACKAGE_VERSION}_source.changes")
      message( STATUS " running for ${DEBUILD_EXECUTABLE} -S ${DEBUILD_OPTIONS}  ")
      message( STATUS " in directory  ${DEBIAN_SOURCE_DIR} ")
      execute_process(
          COMMAND ${DEBUILD_EXECUTABLE} -k${PPA_PGP_KEY} -S ${DEBUILD_OPTIONS} 
          WORKING_DIRECTORY ${DEBIAN_SOURCE_DIR}
      )
      
      message( STATUS "source package for ${RELEASE} generated.")
      
endforeach(RELEASE ${CPACK_DEBIAN_DISTRIBUTION_RELEASES})

##############################################################################
message(STATUS " run  dput ppa:your-lp-id/robocomp ${SOURCE_CHANGES_FILE} from ./Debian for uploading to ppa" )


