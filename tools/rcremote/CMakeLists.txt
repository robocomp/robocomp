cmake_minimum_required(VERSION 2.6)
PROJECT( rcremote )


INSTALL(PROGRAMS "rcremote.py" DESTINATION /opt/robocomp/bin RENAME rcremote )
INSTALL(PROGRAMS "rcremoteserver.py" DESTINATION /opt/robocomp/bin RENAME rcremoteserver )



