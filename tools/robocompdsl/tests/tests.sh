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
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test1/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test1/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_implement.cdsl /tmp/tests/test1/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test1/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test1/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test1_"$language"_require.cdsl /tmp/tests/test1/"$language"/require"
sleep 1

#Test2:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test2/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test2/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_implement.cdsl /tmp/tests/test2/"$language"/implement"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test2/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test2/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test2_"$language"_require.cdsl /tmp/tests/test2/"$language"/require"
sleep 1

#Test3:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test3"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_publish.cdsl /tmp/tests/test3/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test3/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test3/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test3_"$language"_subscribe.cdsl /tmp/tests/test3/"$language"/subscribe"
sleep 1

#Test4:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test4"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_publish.cdsl /tmp/tests/test4/"$language"/publish"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test4/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test4/"$language"/subscribe"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test4_"$language"_subscribe.cdsl /tmp/tests/test4/"$language"/subscribe"
sleep 1

#Test5:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession > /dev/null
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $sess "test5"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_require.cdsl /tmp/tests/test5/"$language"/require"
qdbus org.kde.yakuake /yakuake/sessions splitSessionLeftRight $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement1.cdsl /tmp/tests/test5/"$language"/implement1"
qdbus org.kde.yakuake /yakuake/sessions splitSessionTopBottom $sess > /dev/null
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "mkdir -p /tmp/tests/test5/"$language"/implement2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd /tmp/tests/test5/"$language"/implement2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "robocompdsl /home/robocomp/robocomp/tools/robocompdsl/tests/CDSLs/test5_"$language"_implement2.cdsl /tmp/tests/test5/"$language"/implement2"
sleep 1

echo "running tests for "$language" components"