cmake_minimum_required(VERSION 2.6)
PROJECT( rcdeploymentchecker
        VERSION 2021.2.1
        DESCRIPTION "RoboComp rcdeploymentchecker tool"
        LANGUAGES CXX )

INSTALL(PROGRAMS "rcdeploymentchecker.py" DESTINATION /opt/robocomp/bin RENAME rcdeploymentchecker )