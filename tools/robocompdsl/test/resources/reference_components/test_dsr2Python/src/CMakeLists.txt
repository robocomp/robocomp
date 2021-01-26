cmake_minimum_required(VERSION 2.6)
PROJECT(  pythonAgent )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior  )
