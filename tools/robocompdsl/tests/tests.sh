#/bin/bash
rm -rf /tmp/tests/
echo "Options:"
echo "1 - test for CPP"
echo "2 - test for PYTHON"
echo "3 - Exit"
read -p "Select option [1/2/3]: " number

until [[ $number -gt 0 && $number -lt 4 ]];
do
	echo "bad option"
	read -p "Select option [1/2/3]: " number
done

#CPP
if [ $number -eq 1 ]
then
	language="CPP"
fi

#PYTHON
if [ $number -eq 2 ]
then
	language="PYTHON"
fi

#EXIT
if [ $number -eq 3 ]
then
	exit
fi

###############################################################################################
#Test1:
#CompileCommands
mkdir -p /tmp/tests/test1/"$language"/implement/build
cd /tmp/tests/test1/"$language"/implement
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_implement.cdsl /tmp/tests/test1/"$language"/implement
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//implementCODE|\tprintf("'"Received a call with id=%d"'", id);|g' /tmp/tests/test1/"$language"/implement/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13333|g' /tmp/tests/test1/"$language"/implement/etc/config
	sed -i 's|serviceTest.Endpoints=tcp -p 0|serviceTest.Endpoints=tcp -p 13334|g' /tmp/tests/test1/"$language"/implement/etc/config
	cd /tmp/tests/test1/"$language"/implement/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test1/"$language"/require/build
cd /tmp/tests/test1/"$language"/require
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_require.cdsl /tmp/tests/test1/"$language"/require
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//computeCODE|\tint id;\n\tservicetest_proxy->srvTest(1,id);\n\tprintf("'"Sending a call with id=1"'");|g' /tmp/tests/test1/"$language"/require/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13335|g' /tmp/tests/test1/"$language"/require/etc/config
	sed -i 's|serviceTestProxy = servicetest:tcp -h localhost -p 0|serviceTestProxy = servicetest:tcp -h localhost -p 13334|g' /tmp/tests/test1/"$language"/require/etc/config
	cd /tmp/tests/test1/"$language"/require/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
#RCNODE & ROSCORE (INIT)
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "rcnode/roscore"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "rcnode"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "roscore"
#RCNODE & ROSCORE (END)
#RunCommands
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test1/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/implement --Ice.Config=etc/config"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test1/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/require --Ice.Config=etc/config"
read -rsp $'Press enter to continue...\n'

###############################################################################################
#Test2:
#CompileCommands
mkdir -p /tmp/tests/test2/"$language"/implement/build
cd /tmp/tests/test2/"$language"/implement
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_implement.cdsl /tmp/tests/test2/"$language"/implement
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//implementCODE|\tprintf("'"Received a call with id=%d"'", req.id);|g' /tmp/tests/test2/"$language"/implement/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13323|g' /tmp/tests/test2/"$language"/implement/etc/config
	cd /tmp/tests/test2/"$language"/implement/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test2/"$language"/require/build
cd /tmp/tests/test2/"$language"/require
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_require.cdsl /tmp/tests/test2/"$language"/require
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//computeCODE|\tstd_msgs::Int32 id,idTest;\n\tid.data=1;\n\tservicetest_proxy->srvTest(id,idTest);\n\tprintf("'"Sending a call with id=1"'");|g' /tmp/tests/test2/"$language"/require/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13324|g' /tmp/tests/test2/"$language"/require/etc/config
	cd /tmp/tests/test2/"$language"/require/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
#RunCommands
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test2/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/implement --Ice.Config=etc/config"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test2/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/require --Ice.Config=etc/config"
read -rsp $'Press enter to continue...\n'

###############################################################################################
#Test3:
#CompileCommands
mkdir -p /tmp/tests/test3/"$language"/publish/build
cd /tmp/tests/test3/"$language"/publish
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_publish.cdsl /tmp/tests/test3/"$language"/publish
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//computeCODE|\tint id=1;\n\publishtest_proxy->msgTest(id);\n\tprintf("'"Published id=1"'");|g' /tmp/tests/test3/"$language"/publish/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13313|g' /tmp/tests/test3/"$language"/publish/etc/config
	cd /tmp/tests/test3/"$language"/publish/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test3/"$language"/subscribe/build
cd /tmp/tests/test3/"$language"/subscribe
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_subscribe.cdsl /tmp/tests/test3/"$language"/subscribe
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//subscribesToCODE|\tprintf("'"Received an id=%d"'", id);|g' /tmp/tests/test3/"$language"/subscribe/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13314|g' /tmp/tests/test3/"$language"/subscribe/etc/config
	sed -i 's|publishTestTopic.Endpoints=tcp -p 0|publishTestTopic.Endpoints=tcp -p 12234|g' /tmp/tests/test3/"$language"/subscribe/etc/config
	cd /tmp/tests/test3/"$language"/subscribe/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
#RunCommands
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test3"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/publish --Ice.Config=etc/config"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/subscribe --Ice.Config=etc/config"
read -rsp $'Press enter to continue...\n'

###############################################################################################
#Test4:
#CompileCommands
mkdir -p /tmp/tests/test4/"$language"/publish/build
cd /tmp/tests/test4/"$language"/publish
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_publish.cdsl /tmp/tests/test4/"$language"/publish
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//computeCODE|\tstd_msgs::Int32 id;\n\tid.data=1;\n\tpublishtest_proxy->msgTest(id);\n\tprintf("'"Published id=1"'");|g' /tmp/tests/test4/"$language"/publish/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 11313|g' /tmp/tests/test4/"$language"/publish/etc/config
	cd /tmp/tests/test4/"$language"/publish/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test4/"$language"/subscribe/build
cd /tmp/tests/test4/"$language"/subscribe
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_subscribe.cdsl /tmp/tests/test4/"$language"/subscribe
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//subscribesToCODE|\tprintf("'"Received an id=%d"'", id);|g' /tmp/tests/test4/"$language"/subscribe/src/specificworker.cpp
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 11314|g' /tmp/tests/test4/"$language"/subscribe/etc/config
	cd /tmp/tests/test4/"$language"/subscribe/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
#RunCommands
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test4"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/publish --Ice.Config=etc/config"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/subscribe --Ice.Config=etc/config"
read -rsp $'Press enter to continue...\n'

###############################################################################################
#Test5:
#CompileCommands
mkdir -p /tmp/tests/test5/"$language"/require/build
cd /tmp/tests/test5/"$language"/require
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_require.cdsl /tmp/tests/test5/"$language"/require
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//computeCODE|\tjointmotor1_proxy->setSyncZeroPos();\n\tprintf("'"Sending a call to motor1"'");\n\tjointmotor2_proxy->setSyncZeroPos();\n\tprintf("'"Sending a call to motor2"'");|g' /tmp/tests/test5/"$language"/require/src/specificworker.cpp
	sed -i 's|//namespaces_for_old_components|using namespace RoboCompJointMotor;|g' /tmp/tests/test5/"$language"/require/src/genericworker.h
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13535|g' /tmp/tests/test5/"$language"/require/etc/config
	sed -i 's|JointMotor1Proxy = jointmotor:tcp -h localhost -p 0|JointMotor1Proxy = jointmotor:tcp -h localhost -p 13534|g' /tmp/tests/test5/"$language"/require/etc/config
	sed -i 's|JointMotor2Proxy = jointmotor:tcp -h localhost -p 0|JointMotor2Proxy = jointmotor:tcp -h localhost -p 13536|g' /tmp/tests/test5/"$language"/require/etc/config
	cd /tmp/tests/test5/"$language"/require/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test5/"$language"/implement1/build
cd /tmp/tests/test5/"$language"/implement1
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement1.cdsl /tmp/tests/test5/"$language"/implement1
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//implementCODE|\tprintf("'"Received a call for motor1"'");|g' /tmp/tests/test5/"$language"/implement1/src/specificworker.cpp
	sed -i 's|//namespaces_for_old_components|using namespace RoboCompJointMotor;|g' /tmp/tests/test5/"$language"/implement1/src/genericworker.h
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13533|g' /tmp/tests/test5/"$language"/implement1/etc/config
	sed -i 's|JointMotor.Endpoints=tcp -p 0|JointMotor.Endpoints=tcp -p 13534|g' /tmp/tests/test5/"$language"/implement1/etc/config
	cd /tmp/tests/test5/"$language"/implement1/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
mkdir -p /tmp/tests/test5/"$language"/implement2/build
cd /tmp/tests/test5/"$language"/implement2
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement2.cdsl /tmp/tests/test5/"$language"/implement2
#CPPsed
if [ "$language" == "CPP" ]
then
	sed -i 's|//implementCODE|\tprintf("'"Received a call for motor2"'");|g' /tmp/tests/test5/"$language"/implement2/src/specificworker.cpp
	sed -i 's|//namespaces_for_old_components|using namespace RoboCompJointMotor;|g' /tmp/tests/test5/"$language"/implement2/src/genericworker.h
	sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13538|g' /tmp/tests/test5/"$language"/implement2/etc/config
	sed -i 's|JointMotor.Endpoints=tcp -p 0|JointMotor.Endpoints=tcp -p 13536|g' /tmp/tests/test5/"$language"/implement2/etc/config
	cd /tmp/tests/test5/"$language"/implement2/build
	cmake .. && make -j3
fi
#PYTHONsed
if [ "$language" == "PYTHON" ]
then
	echo "por hacer!!"
	cmake . && make -j3
fi
#RunCommands
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
term=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeTerminalId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test5"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/implement1 --Ice.Config=etc/config"
qdbus org.kde.yakuake /yakuake/sessions splitSessionTopBottom $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "./bin/implement2 --Ice.Config=etc/config"
sleep 1 #require must be executed after implements
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal $term "cd /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal $term "./bin/require --Ice.Config=etc/config"
read -rsp $'Press enter to continue...\n'
echo "tests for "$language" components finished"