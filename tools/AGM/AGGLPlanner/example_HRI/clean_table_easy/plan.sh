#!/usr/bin/env bash

e=$PWD
cd ~/AGM
make && sudo make install
cd $e
rm -f *.pyc
aggl2agglpy -i HRI.aggl -a active.aggl.py -f full.aggl.py > /dev/null
aggl2agglpy -x target3.xml -o target.py > /dev/null
echo "-------------------------"
echo "-- planning starts now --"
echo "-------------------------"
time agglplanner active.aggl.py init.xml target.py










# 
# e=$PWD
# cd ~/AGM
# make && sudo make install
# 
# cd $e
# rm -f *.pyc
# time aggl2agglpy -i $1 -a active.aggl.py -f full.aggl.py
# time aggl2agglpy -x $3 -o target.py
# time agglplanner active.aggl.py $2 target.py
# 
# 
# 
