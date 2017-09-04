#!/usr/bin/sh

sudo apt-get install python-pyparsing python-pyside pyside-tools libpython2.7-dev python-dev libboost-all-dev cmake python-imaging python-numpy pypy cython libgsl-dev libopenscenegraph-dev pypy-setuptools python-setuptools libxml2-dev python-pygraphviz python-networkx pypy-dev python-thrift libqt4-dev

git clone https://github.com/eleme/thriftpy.git
cd thriftpy
sudo pypy setup.py install
sudo make clean
sudo python setup.py install
git clone https://bitbucket.org/pypy/numpy.git
cd numpy
sudo pypy setup.py install
