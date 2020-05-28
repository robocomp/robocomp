cmake_minimum_required( VERSION 2.6 )


INSTALL(PROGRAMS "workspace.py" DESTINATION /opt/robocomp/python)
INSTALL(PROGRAMS "rc_init_ws.py" DESTINATION /opt/robocomp/bin RENAME rc_init_ws )
INSTALL(PROGRAMS "rcrun.py" DESTINATION /opt/robocomp/bin RENAME rcrun )
INSTALL(PROGRAMS "rced.py" DESTINATION /opt/robocomp/bin RENAME rced )
INSTALL(PROGRAMS "rcbuild.py" DESTINATION /opt/robocomp/bin RENAME rcbuild )
INSTALL(PROGRAMS "rccomp.py" DESTINATION /opt/robocomp/bin RENAME rccomp )
INSTALL(PROGRAMS "rccd_utils.py" DESTINATION /opt/robocomp/bin RENAME rccd_utils )
INSTALL(PROGRAMS "rccd.sh" DESTINATION /opt/robocomp/share ) #source this in .bashrc
INSTALL(CODE "execute_process(COMMAND sh setup.sh)")

