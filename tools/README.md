## RoboComp development tools

This folder contains a set of tools created to facilitate the process of software development with RoboComp. Detailed information on each tool can be found in the corresponding README.md files.


### Yakuake script


This is a simple script to automatically start RoboComp's favourite terminal tool, [Yakuake](https://yakuake.kde.org/), with two named tabs running Ice's pub/sub broker Storm through our *rcnode* script and a component named *comp1*.

```sh
#!/bin/bash
pkill -f icebox&
pkill -f rcremoteserver&

tabs_to_close="rcnode rcmanager rcremoteserver"

tabs_to_open="rcnode rcremoteserver"

echo "removing"
#Remove previous tabs if exist, avoiding to create multiple instances of these
for specific_tab in $tabs_to_close
do
	found=0

	#loop over all open tabs
	for sessid in $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.sessionIdList | tr ',' '\n')
	do
		# get the name of the tab
		name=$(qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.tabTitle $sessid)		
		if [ "$name" = $specific_tab ] 
		then
			found=1
			qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.removeSession $sessid
		fi	
	done 		
done

echo "creating"
#loop to open specific tabs
for specific_tab in $tabs_to_open
do
	#create new session
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
	#get id of open session
	sess0=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
	#run command on active session
	qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand $specific_tab
	#change the name of session
	qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess0 $specific_tab
done
```

Additional blocks can be added to create and initialize new tabs in Yakuake.

Please, copy this code to a .sh file and execute it whenever you need to start your multi-component development environment.
