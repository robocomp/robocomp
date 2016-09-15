DOMAINNAME=domain.aggl

echo
planningTest="navigation/hallToPatio"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target.xml
cd -
echo "######################################################################"
sleep 1


echo
echo "######################################################################"
planningTest="perception/findGranny"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="grasp"
echo "Next: $planningTest 1"
cd $planningTest
agglplan ../../../etc/$DOMAINNAME initialModel1.xml target1.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="grasp"
echo "Next: $planningTest 2"
cd $planningTest
agglplan ../../../etc/$DOMAINNAME initialModel2.xml target2.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 0"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target0.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 1"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target1.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 2"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target2.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="deliver/moveObject"
echo "Next: $planningTest 2"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target.xml
cd -
echo "######################################################################"
sleep 1


echo
planningTest="hri/coffee 0"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target0.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 1"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target1.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 2"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target2.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 3"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target3.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 4"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target4.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 5"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target5.xml
cd -
echo "######################################################################"
sleep 1

echo
planningTest="hri/coffee 6"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml target6.xml
cd -
echo "######################################################################"
sleep 1



