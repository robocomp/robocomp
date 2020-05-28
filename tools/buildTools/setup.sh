#!/usr/bin/bash

bashrc_code="
if [ -f /opt/robocomp/share/rccd.sh ]; then
    . /opt/robocomp/share/rccd.sh
fi"

if [ -f ~/.bashrc ]; then
    exist=$(cat ~/.bashrc | grep "/opt/robocomp/share/rccd.sh")
    if [ -z "$exist" ]; then
      echo "$bashrc_code" >> ~/.bashrc
    else
      echo "Installation of rccd already exist in ~/.bashrc"
    fi
fi

