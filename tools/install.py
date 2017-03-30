# -*- coding: utf-8 -*-
"""
Created on Sat Feb 14 19:36:39 2015

@author: pbustos
"""

import os, sys, pprint, subprocess
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

home = os.getenv("HOME")

os.chdir(home)

call(["git","clone","https://github.com/robocomp/robocomp.git"])

call(["sudo","ln","-s","/opt/robocomp/-1.0","/opt/robocomp"])
call(["sudo","ln","-s",home,"/home/robocomp"])

call(["echo","export", os.path.join('ROBOCOMP=',home,'robocomp'),">>",os.path.join(home,'.bashrc')])
call(["echo","export PATH=$PATH:/opt/robocomp/bin",">>",os.path.join(home,'.bashrc')])

cmd = ['bash','-c','source .bashrc']

proc = subprocess.Popen(cmd, stdout = subprocess.PIPE)

for line in proc.stdout:
  (key, _, value) = line.partition("=")
  os.environ[key] = value

proc.communicate()


os.chdir(os.path.join(home,'robocomp'))

call(["cmake","."])
call(["make"])
call(["sudo","make","install"])

call(["sudo","echo","/opt/robocomp/lib",">>","/etc/ld.so.conf"])
call(["sudo","ldconfig"])
