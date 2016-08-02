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

#Test1:
#CompileCommands
mkdir -p /tmp/tests/test1/"$language"/implement/build
cd /tmp/tests/test1/"$language"/implement
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_implement.cdsl /tmp/tests/test1/"$language"/implement
sed -i 's|//implementCODE|\tprintf("'"Received a call with id=%d"'", id);|g' /tmp/tests/test1/"$language"/implement/src/specificworker.cpp
sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13333|g' /tmp/tests/test1/"$language"/implement/etc/config
sed -i 's|serviceTest.Endpoints=tcp -p 0|serviceTest.Endpoints=tcp -p 13334|g' /tmp/tests/test1/"$language"/implement/etc/config
cd /tmp/tests/test1/"$language"/implement/build
cmake ..
make -j3
mkdir -p /tmp/tests/test1/"$language"/require/build
cd /tmp/tests/test1/"$language"/require
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_require.cdsl /tmp/tests/test1/"$language"/require
sed -i 's|//computeCODE|\tint id;\n\tservicetest_proxy->srvTest(1,id);\n\tprintf("'"Sending a call with id=1"'");|g' /tmp/tests/test1/"$language"/require/src/specificworker.cpp
sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13335|g' /tmp/tests/test1/"$language"/require/etc/config
sed -i 's|serviceTestProxy = servicetest:tcp -h localhost -p 0|serviceTestProxy = servicetest:tcp -h localhost -p 13334|g' /tmp/tests/test1/"$language"/require/etc/config
cd /tmp/tests/test1/"$language"/require/build
cmake ..
make -j3
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
#Test2:
#CompileCommands
mkdir -p /tmp/tests/test2/"$language"/implement/build
cd /tmp/tests/test2/"$language"/implement
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_implement.cdsl /tmp/tests/test2/"$language"/implement
sed -i 's|//implementCODE|\tprintf("'"Received a call with id=%d"'", req.id);|g' /tmp/tests/test2/"$language"/implement/src/specificworker.cpp
sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13323|g' /tmp/tests/test2/"$language"/implement/etc/config
cd /tmp/tests/test2/"$language"/implement/build
cmake ..
make -j3
mkdir -p /tmp/tests/test2/"$language"/require/build
cd /tmp/tests/test2/"$language"/require
robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_require.cdsl /tmp/tests/test2/"$language"/require
sed -i 's|//computeCODE|\tstd_msgs::Int32 id,idTest;\n\tid.data=1;\n\tservicetest_proxy->srvTest(id,idTest);\n\tprintf("'"Sending a call with id=1"'");|g' /tmp/tests/test2/"$language"/require/src/specificworker.cpp
sed -i 's|CommonBehavior.Endpoints=tcp -p 1|CommonBehavior.Endpoints=tcp -p 13324|g' /tmp/tests/test2/"$language"/require/etc/config
cd /tmp/tests/test2/"$language"/require/build
cmake ..
make -j3
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

#Test3:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test3"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test3/"$language"/publish/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_publish.cdsl /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test3/"$language"/subscribe/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_subscribe.cdsl /tmp/tests/test3/"$language"/subscribe"
read -rsp $'Press enter to continue...\n'

#Test4:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test4"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test4/"$language"/publish/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_publish.cdsl /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test4/"$language"/subscribe/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_subscribe.cdsl /tmp/tests/test4/"$language"/subscribe"
read -rsp $'Press enter to continue...\n'

#Test5:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test5"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/require/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_require.cdsl /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/implement1/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement1.cdsl /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions splitSessionTopBottom $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/implement2/build"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement2.cdsl /tmp/tests/test5/"$language"/implement2"
read -rsp $'Press enter to continue...\n'

echo "tests for "$language" components finished"