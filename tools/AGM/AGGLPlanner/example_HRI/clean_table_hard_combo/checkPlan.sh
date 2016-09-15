#!/usr/bin/env bash

e=$PWD
cd ~/AGM
make && sudo make install
cd $e
rm -f *.pyc
aggl2agglpy -i HRI.aggl -a active.aggl.py -f full.aggl.py
aggl2agglpy -x target3.xml -o target.py
agglplanchecker active.aggl.py init.xml target3.plan target.py result.xml




# # ./checkPlan.sh HRI.aggl init.xml target3.plan target3.xml result.xml
# e=$PWD
# cd ~/AGM
# make && sudo make install
# cd $e
# rm -f *.pyc
# aggl2agglpy -i $1 -a active.aggl.py -f full.aggl.py
# aggl2agglpy -x $4 -o target.py
# agglplanchecker active.aggl.py $2 $3 target.py $5
# #1 Domain AGGL
# #2 Init XML
# #3 Plan LIST
# #4 Target XML
# #5 Result XML - optional
# 




