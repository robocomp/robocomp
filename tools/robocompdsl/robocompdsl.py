#!/usr/bin/env python
import sys

if sys.version_info >= (3, 0):
    sys.stdout.write("RoboCompDSL requires Python 2.x, not Python 3.x\n")
    sys.exit(1)

sys.path.append('/opt/robocomp/python')
import robocompdsl_core
robocompdsl_core.main(sys.argv)

