
cmake_minimum_required(VERSION 2.6)
PROJECT(  rcmaster )

macro(makeLink src dest target)
  	add_custom_command(TARGET ${target} PRE_BUILD
    COMMAND ln -sf ${src} {dest} DEPENDS ${dest} COMMENT "mklink ${src} -> ${dest}")
endmacro()

# Program destination directory
SET( DEST_DIR "/opt/robocomp")

INSTALL(DIRECTORY "src/" DESTINATION "${DEST_DIR}/share/rcmaster" )
makeLink( "${DEST_DIR}/share/rcmaster/rcmaster.py" "${DEST_DIR}/bin/rcmaster" "${DEST_DIR}/bin/rcmaster")
