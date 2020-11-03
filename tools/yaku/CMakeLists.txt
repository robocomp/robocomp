cmake_minimum_required( VERSION 2.8)

INSTALL(PROGRAMS "yaku/yaku.py" DESTINATION /opt/robocomp/bin RENAME yaku )
INSTALL(DIRECTORY "pyaku" DESTINATION /opt/robocomp/python)