cmake_minimum_required( VERSION 3.10 )
PROJECT( rcis
        VERSION 2021.2.1
        DESCRIPTION "RoboComp RCInnerModelSimulator tool"
        LANGUAGES CXX )

SUBDIRS( src )

#ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
#ADD_CUSTOM_TARGET(installdoc mkdir -p /opt/robocomp/doc COMMAND cp -R doc/html /opt/robocomp/doc/rcis )
