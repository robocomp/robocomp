#!/usr/bin/bash
set -e
rccd_code="
if [ -f /opt/robocomp/share/rccd.sh ]; then
    source /opt/robocomp/share/rccd.sh
fi
"

source_file="/etc/profile.d/robocomp.sh"
if [ ! -f $source_file ]; then
	echo "###" >> $source_file
fi
if [ -f $source_file ]; then
    exist=$(cat $source_file | grep "/opt/robocomp/share/rccd.sh" || echo "")
    echo "Contenido $exist"
    if [ -z $exist ]; then
      echo "rccd code instaled in $source_file"
      echo "$rccd_code" >> $source_file
    else
      echo "Installation of rccd already exist in $source_file"
    fi
fi

