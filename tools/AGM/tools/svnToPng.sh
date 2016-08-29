#!/bin/bash

for i in *.svg
do
	o=`echo $i | sed -e 's/svg$/png/'`
	echo "$i --> $o"
	echo "  inkscape"
	inkscape $i --export-dpi=150  --export-png=$o > /dev/null
	echo "  convert"
	convert -trim $o $o
done

