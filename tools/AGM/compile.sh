#!/bin/sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:.


# Should we compile with RoboComp support?
finished=""
while [ "$finished" = "" ]; do
	case $ROBOCOMP_SUPPORT in
		y | Y | yes | YES ) ROBOCOMP_SUPPORT="True"; finished="true";;
		n | N | no | NO ) ROBOCOMP_SUPPORT="False"; finished="true";;
		*) ROBOCOMP_SUPPORT="-1";
			echo -n "Do you want RoboComp support? (y/n): "
			read ROBOCOMP_SUPPORT;;
	esac
done

# Should we compile Python bindings?
#finished=""
#while [ "$finished" = "" ]; do
#	case $PYTHON_BINDINGS in
#		y | Y | yes | YES ) PYTHON_BINDINGS="True"; finished="true";;
#		n | N | no | NO ) PYTHON_BINDINGS="False"; finished="true";;
#		*) PYTHON_BINDINGS="-1";
#			echo -n "Do you want to build Python bindings? (y/n): "
#			read PYTHON_BINDINGS;;
#	esac
#done
PYTHON_BINDINGS="False"

mkdir -p build
cd build && pwd && cmake .. -DROBOCOMP_SUPPORT=${ROBOCOMP_SUPPORT} -DPYTHON_BINDINGS=${PYTHON_BINDINGS} && make -j4 && sudo make install

