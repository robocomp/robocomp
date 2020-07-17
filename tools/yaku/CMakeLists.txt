cmake_minimum_required( VERSION 2.8)

INSTALL(PROGRAMS "yaku.py" DESTINATION /opt/robocomp/bin RENAME yaku )
INSTALL(FILES "yaku_lib.py" DESTINATION /opt/robocomp/python)