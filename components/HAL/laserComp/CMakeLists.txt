cmake_minimum_required( VERSION 2.6 )

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )

SUBDIRS( src )

INSTALL(FILES etc/config DESTINATION ${CONFIG_INSTALL_DIR} RENAME laser.conf )

ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
ADD_CUSTOM_TARGET(installdoc mkdir -p ${DOC_INSTALL_DIR} COMMAND cp -R doc/html ${DOC_INSTALL_DIR}/laserComp )

