cmake_minimum_required( VERSION 2.8)
project(robocompdsl)


#  Installation  #
##################


INSTALL(PROGRAMS "robocompdsl.py" DESTINATION /opt/robocomp/bin RENAME robocompdsl )
INSTALL(DIRECTORY "templates" DESTINATION /opt/robocomp/python/${PROJECT_NAME} )
INSTALL(DIRECTORY "dsl_parsers" DESTINATION /opt/robocomp/python/${PROJECT_NAME} )
INSTALL(FILES "filesgenerator.py" DESTINATION /opt/robocomp/python/${PROJECT_NAME} )
INSTALL(FILES "robocompdslutils.py" DESTINATION /opt/robocomp/python/${PROJECT_NAME} )
INSTALL(FILES "rcExceptions.py" DESTINATION /opt/robocomp/python/${PROJECT_NAME})

