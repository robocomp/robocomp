cmake_minimum_required( VERSION 2.8)


#  Installation  #
##################

INSTALL(PROGRAMS "rcExceptions.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "robocompdsl.py" DESTINATION /opt/robocomp/bin RENAME robocompdsl )

INSTALL(DIRECTORY "templates" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(DIRECTORY "dsl_parsers" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(FILES "TEMPLATE.ICE" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(FILES "componentgenerator.py" DESTINATION /opt/robocomp/python )
INSTALL(FILES "robocompdslutils.py" DESTINATION /opt/robocomp/python )
#INSTALL(DIRECTORY "cogapp" DESTINATION /opt/robocomp/python )
INSTALL(DIRECTORY "dsl_parsers" DESTINATION /opt/robocomp/python )
INSTALL(FILES "rcExceptions.py" DESTINATION /opt/robocomp/python )
