cmake_minimum_required( VERSION 2.8)


#  Installation  #
##################

INSTALL(PROGRAMS "parseCDSL.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "parseSMDSL.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "parseIDSL.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "rcExceptions.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "robocompdsl_core.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "robocompdsl.py" DESTINATION /opt/robocomp/bin RENAME robocompdsl )

INSTALL(DIRECTORY "templateCPP" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(DIRECTORY "templatePython" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(FILES "TEMPLATE.ICE" DESTINATION /opt/robocomp/share/robocompdsl )
INSTALL(DIRECTORY "cogapp" DESTINATION /opt/robocomp/python )
INSTALL(FILES "rcExceptions.py" DESTINATION /opt/robocomp/python )
