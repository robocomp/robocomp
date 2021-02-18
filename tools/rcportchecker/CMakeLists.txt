cmake_minimum_required(VERSION 3.10)
PROJECT( rcportchecker
        VERSION 2021.2.1
        DESCRIPTION "RoboComp rcportchecker tool"
        LANGUAGES CXX )

INSTALL(PROGRAMS "rcportchecker.py" DESTINATION /opt/robocomp/bin RENAME rcportchecker )
INSTALL(FILES "rcportchecker.py" DESTINATION /opt/robocomp/python )
