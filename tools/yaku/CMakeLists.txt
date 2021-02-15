cmake_minimum_required( VERSION 3.10 )
project(yaku
        VERSION 1.0.0
        DESCRIPTION "Robocomp yaku utility" )

install(PROGRAMS "yaku/yaku.py" DESTINATION /opt/robocomp/bin RENAME yaku )
install(DIRECTORY "pyaku" DESTINATION /opt/robocomp/python)