#!/usr/bin/env bash
INIT=init
TARGET=target
DOMAIN=HRI
e=$PWD
cd ~/AGM
make && sudo make install
cd $e
# rm -f *.pyc *.py *~
# aggl2agglpy -i ${DOMAIN}.aggl -a active.aggl.py -f full.aggl.py > /dev/null
# aggl2agglpy -x ${TARGET}.xml -o target.py > /dev/null
agm_xmlViewer ${INIT}.xml ${TARGET}.xml
echo "-- planning starts now --"
STARTTIME=$(date +%s)
agglplanner active.aggl.py ${INIT}.xml ${TARGET}.py result.plan
ENDTIME=$(date +%s)
echo "It took $(($ENDTIME - $STARTTIME)) seconds to generate the plan."
echo "Verifying the generated plan..."
agglplanchecker active.aggl.py ${INIT}.xml result.plan target.py result.xml
agm_xmlViewer result.xml


