cmake_minimum_required( VERSION 3.10 )

SUBDIRS( src )

ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
ADD_CUSTOM_TARGET(installdoc mkdir -p /opt/robocomp/doc COMMAND cp -R doc/html /opt/robocomp/doc/rcis )
