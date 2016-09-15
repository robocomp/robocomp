#!/bin/sh
doxygen=$2
inputpath=$1

echo "doxygen path: $doxygen"
echo "input path: $inputpath"


rm -rf doc/html

export ROBOCOMP_SUPPORT=1
$doxygen doc/Doxyfile

cp $inputpath/doc/tabs.css doc/html/tabs.css
cp $inputpath/doc/AGGLEditor.png doc/html/AGGLEditor.png
cp $inputpath/doc/AGGLEditor_A.png doc/html/AGGLEditor_A.png
cp $inputpath/doc/AGGLEditor_B.png doc/html/AGGLEditor_B.png
cp $inputpath/doc/wholePicture.png doc/html/wholePicture.png
cp $inputpath/doc/init0.png doc/html/init0.png
cp $inputpath/doc/init2.png doc/html/init2.png
cp $inputpath/doc/goal2.png doc/html/goal2.png



