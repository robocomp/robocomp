cmake_minimum_required(VERSION 2.6)
PROJECT(  ${component_name} )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior ${ifaces_list} )
