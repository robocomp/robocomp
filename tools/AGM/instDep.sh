#!/usr/bin/sh


sudo apt-get install python-pyparsing python-pyside pyside-tools libpython2.7-dev python-dev libboost-all-dev cmake python-imaging python-numpy pypy pypy-setuptools cython

git clone https://github.com/eleme/thriftpy.git
cd thriftpy
sudo pypy setup.py install
sudo make clean
sudo python setup.py install


