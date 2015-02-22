# -*- coding: utf-8 -*-
"""
Created on Sat Feb 14 19:36:39 2015

@author: pbustos
"""

import os, sys
from subprocess import call

modules = ["git","git-annex","cmake","g++","libgsl0-dev","libopenscenegraph-dev","cmake-qt-gui",
	       "zeroc-ice35","freeglut3-dev","libboost-dev","libboost-thread-dev","qt4-dev-tools",
		   "yakuake","openjdk-7-jre","kdevelop","vim", "aptitude"]

print "updating repository..."
call(["sudo", "apt-get", "update"])

print "installing packages..."
command = ["sudo", "apt-get", "install"]
command.extend(modules)
call(command)

call(["cd"])
call(["git","clone","https://github.com/robocomp/robocomp.git"])

call(["sudo","ln","-s","/opt/robocomp/-1.0","/opt/robocomp"])
call(["sudo","ln","-s","/home/username","/home/robocomp"])

call(["echo","export ROBOCOMP=/home/username/robocomp",">>","/home/usermane/.bashrc"])
call(["echo","export PATH=$PATH:/opt/robocomp/bin",">>","/home/usermane/.bashrc"])
call(["source",".bashrc"])

call(["cd","robocomp"])
call(["cmake","."])
call(["make"])
call(["sudo","make","install"])

call(["sudo","echo","/opt/robocomp/lib",">>","/etc/ld.so.conf"])
call(["sudo","ldconfig"])

#