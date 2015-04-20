#!/usr/bin/env bash

binary=$1
cwdv=$2
tabname=$3
params="${4} ${5} ${6} ${7} ${8} ${9} ${11} ${12} ${13} ${14} ${15} ${16} ${17} ${18} ${19} ${20}"

echo "yo" $0
echo "bin" $binary
echo "cwd" $cwdv
echo "tabname" $tabname
echo "params" $params

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "cd ${cwdv}"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "make -j1"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "$binary $params"
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess $tabname

