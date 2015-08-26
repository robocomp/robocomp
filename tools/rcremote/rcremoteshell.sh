#!/usr/bin/env bash


# The documentation is in qdbusviewer. To install: sudo apt-get install qt5-default qttools5-dev-tools
binary=$1
cwdv=$2
tabname=$3
params="${4} ${5} ${6} ${7} ${8} ${9} ${11} ${12} ${13} ${14} ${15} ${16} ${17} ${18} ${19} ${20}"
flag=0

echo "yo" $0
echo "bin" $binary
echo "cwd" $cwdv
echo "tabname" $tabname
echo "params" $params

#This is for split the sessionIDList string of the tabs in a list (yeah!) 
sessionList=$(echo `qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.sessionIdList` | tr "," "\n")

for sessionID in $sessionList
do
	sessionName=`qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.tabTitle $sessionID`
# 	if [ $sessionName == $tabname ]; then
	if [ "$sessionName" = "$tabname" ]; then
		flag=1
		session=`expr $sessionID + 1`
		processID=`qdbus org.kde.yakuake /Sessions/$session org.kde.konsole.Session.processId`
		fourGroundProcessID=`qdbus org.kde.yakuake /Sessions/$session org.kde.konsole.Session.foregroundProcessId`
      
		# If the ids are the same, the user process is death
		if [ "$processID" = "$fourGroundProcessID" ]; then
			qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal $sessionID "$binary $params"		
			qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal $sessionID "echo $flag"
			qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal $sessionID "make -j1"
		fi
		
	#ELSE LANZAR UNA EXCEPCION
	fi
done

if [ $flag == 0 ]; then
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
	sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd ${cwdv}"
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "echo $flag"
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "make -j1"
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "$binary $params"
	qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess $tabname
fi
