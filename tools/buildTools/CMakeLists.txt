cmake_minimum_required( VERSION 2.6 )


INSTALL(PROGRAMS "workspace.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "rc_init_ws.py" DESTINATION /opt/robocomp/bin RENAME rc_init_ws )
INSTALL(PROGRAMS "rcrun.py" DESTINATION /opt/robocomp/bin RENAME rcrun )
INSTALL(PROGRAMS "rcedit.py" DESTINATION /opt/robocomp/bin RENAME rcedit )
INSTALL(PROGRAMS "rcbuild.py" DESTINATION /opt/robocomp/bin RENAME rcbuild )
INSTALL(PROGRAMS "setup.sh" DESTINATION /opt/robocomp/share ) #source this in .bashrc
