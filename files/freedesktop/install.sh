xdg-icon-resource install --mode system robocomp-icon.png --size 128 RoboComp-lo

cp RoboComp.menu /etc/xdg/menus/applications-merged/
cp RoboComp-RoboComp.directory /usr/share/desktop-directories

for file in `ls *.desktop`; do
	echo "\nInstalling $file"
	xdg-desktop-menu install --mode system $file
	xdg-desktop-menu forceupdate --mode system $file
	echo ""
done


