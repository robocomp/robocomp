cmake_minimum_required( VERSION 3.10 )
find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print (get_python_lib())" OUTPUT_VARIABLE PYTHON_SITE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
INSTALL(FILES "robocomp.pth" DESTINATION ${PYTHON_SITE_PATH} )


#INSTALL(PROGRAMS "rced.py" DESTINATION /opt/robocomp/bin RENAME rced )
#INSTALL(PROGRAMS "rccomp.py" DESTINATION /opt/robocomp/bin RENAME rccomp )
#INSTALL(PROGRAMS "rcfind.py" DESTINATION /opt/robocomp/bin RENAME rcfind )
INSTALL(CODE "message(STATUS \"Installing rccd in ~/.bashrc and ~/.zshrc\")
execute_process(COMMAND sh \"${CMAKE_CURRENT_LIST_DIR}/setup.sh\")")
INSTALL(CODE "message(STATUS \"Installing autocompletion in ~/.bashrc\")
execute_process(COMMAND activate-global-python-argcomplete)")

execute_process(COMMAND rcworkspace add ${CMAKE_SOURCE_DIR}/components/ --accept-all)
