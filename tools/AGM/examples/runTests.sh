# echo
# planningTest="makeMeCoffee/hri/coffee 0"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target0.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee0.py
# cp /tmp/target.py targetCoffee0.py
# 
# echo
# planningTest="makeMeCoffee/hri/coffee 1"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target1.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee1.py
# cp /tmp/target.py targetCoffee1.py

# echo
# planningTest="makeMeCoffee/hri/coffee 2"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target2.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee2.py


# echo
# planningTest="makeMeCoffee/hri/coffee 3"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target3.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee3.py
# cp /tmp/target.py targetCoffee3.py

# echo
# planningTest="makeMeCoffee/hri/coffee 4"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target4.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee4.py
# cp /tmp/target.py targetCoffee4.py


# echo
# planningTest="makeMeCoffee/hri/coffee 5"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target5.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee5.py
# cp /tmp/target.py targetCoffee5.py
# 
# echo
# planningTest="makeMeCoffee/hri/coffee 6"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target6.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee6.py
# cp /tmp/target.py targetCoffee6.py
# 
# 
# echo
# planningTest="makeMeCoffee/experiment"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../domain.aggl initialModelCOFFEE.xml targetModelCOFFEE.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainCoffee1.py
# cp /tmp/target.py targetCoffee1.py


echo
planningTest="basic"
echo "Next: $planningTest"
cd $planningTest
agglplan grammar.aggl initialModel.xml targetModel.xml
cd -
echo "######################################################################"
sleep 1
cp /tmp/domain.py domainBasic.py
cp /tmp/target.py targetBasic.py

# echo
# planningTest="logistics"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan domain1_withCombo.aggl init.xml goal.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainLogistics.py
# cp /tmp/target.py targetLogistics.py
# 
# 
# echo
# planningTest="logistics 0"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan domain1_withCombo.aggl init0.xml goal0.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainLogistics0.py
# cp /tmp/target.py targetLogistics0.py
# 
# 
# echo
# planningTest="logistics 1"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan domain1_withCombo.aggl init1.xml goal1.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainLogistics1.py
# cp /tmp/target.py targetLogistics1.py
# 
# 
# echo
# planningTest="logistics 2"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan domain1_withCombo.aggl init2.xml goal2.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainLogistics2.py
# cp /tmp/target.py targetLogistics2.py
# 
# 
# echo
# planningTest="makeMeCoffee/navigation/hallToPatio"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainHallToPatio.py
# cp /tmp/target.py targetHallToPatio.py
# 
# 
# echo
# echo "######################################################################"
# planningTest="makeMeCoffee/perception/findGranny"
# echo "Next: $planningTest"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainFindGranny.py
# cp /tmp/target.py targetFindGranny.py
# 
# 
# echo
# planningTest="makeMeCoffee/grasp"
# echo "Next: $planningTest 1"
# cd $planningTest
# agglplan ../domain.aggl initialModel1.xml target1.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainGrasp1.py
# cp /tmp/target.py targetGrasp1.py
# 
# 
# echo
# planningTest="makeMeCoffee/grasp"
# echo "Next: $planningTest 2"
# cd $planningTest
# agglplan ../domain.aggl initialModel2.xml target2.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainGrasp2.py
# cp /tmp/target.py targetGrasp2.py
# 
# 
# echo
# planningTest="makeMeCoffee/deliver/deliverKnown"
# echo "Next: $planningTest 0"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target0.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainDeliver0.py
# cp /tmp/target.py targetDeliver0.py
# 
# 
# echo
# planningTest="makeMeCoffee/deliver/deliverKnown"
# echo "Next: $planningTest 1"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target1.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainDeliver1.py
# cp /tmp/target.py targetDeliver1.py
# 
# 
# echo
# planningTest="makeMeCoffee/deliver/deliverKnown"
# echo "Next: $planningTest 2"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target2.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainDeliver2.py
# cp /tmp/target.py targetDeliver2.py
# 
# 
# echo
# planningTest="makeMeCoffee/deliver/moveObject"
# echo "Next: $planningTest 2"
# cd $planningTest
# agglplan ../../domain.aggl initialModel.xml target.xml
# cd -
# echo "######################################################################"
# sleep 1
# cp /tmp/domain.py domainDeliver2.py
# cp /tmp/target.py targetDeliver2.py
# 
# 
