#!/bin/bash

bashrc_code="
if [ -f /opt/robocomp/share/rccd.sh ]; then
    source /opt/robocomp/share/rccd.sh
fi
"

bashrc_code_rbcd="
if [ -f /opt/robocomp/share/rbcd.sh ]; then
    . /opt/robocomp/share/rbcd.sh
fi"

zshrc_code_rbcd="
if [ -f /opt/robocomp/share/rbcd.zsh ]; then
    . /opt/robocomp/share/rbcd.zsh
fi"

if [ -f ~/.bashrc ]; then
    exist=$(cat ~/.bashrc | grep "/opt/robocomp/share/rccd.sh")
    if [ -z "$exist" ]; then
      echo "rccd code installed in ~/.bashrc"
      echo "$bashrc_code" >> ~/.bashrc
    else
      echo "Installation of rccd already exist in $source_file"
    fi

    exist_rbcd=$(cat ~/.bashrc | grep "/opt/robocomp/share/rbcd.sh")
    if [ -z "$exist_rbcd" ]; then
      echo "rbcd code installed in ~/.bashrc"
      echo "$bashrc_code_rbcd" >> ~/.bashrc
    else
      echo "Installation of rbcd already exist in ~/.bashrc"
    fi
fi

if [ -f ~/.zshrc ]; then
    exist=$(cat ~/.zshrc | grep "/opt/robocomp/share/rccd.sh")
    if [ -z "$exist" ]; then
      echo "rccd code installed in ~/.zshrc"
      echo "$bashrc_code" >> ~/.zshrc
    else
      echo "Installation of rccd already exist in ~/.zshrc"
    fi

    exist_rbcd=$(cat ~/.zshrc | grep "/opt/robocomp/share/rbcd.zsh")
    if [ -z "$exist_rbcd" ]; then
      echo "rbcd code installed in ~/.zshrc"
      echo "$zshrc_code_rbcd" >> ~/.zshrc
    else
      echo "Installation of rbcd already exist in ~/.zshrc"
    fi
fi


