#!/bin/bash

#Checks if script is run as root or not 
if test "`id -u`" -ne 0
    then 
    echo "You need to run this script as root!" 
    exit 
fi

NU=$(who am i | awk '{print $1}');
sudo ln -s /home/$NU /home/robocomp;

num=$(grep -c "ROBOCOMP=/home/$NU/robocomp" ~/.bashrc)
if [ $num -eq 0 ]; then
    echo "export ROBOCOMP=/home/$NU/robocomp" >> ~/.bashrc;
    echo "added ROBOCOMP env varible to bashrc";
fi

num=$(grep -c "PATH=$PATH:/opt/robocomp/bin" ~/.bashrc)
if [ $num -eq 0 ]; then
    echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc;
    echo "added /opt/robocomp/bin to PATH ";
fi

source ~/.bashrc

#as the script is run as root export the env variables
export ROBOCOMP=/home/$NU/robocomp
export PATH=$PATH:/opt/robocomp/bin

cd ~/robocomp
cmake . && make

if [ $? -eq 0 ]; then 
    sudo make install
fi

if [ $? -eq 0 ]; then 
    num=$(grep -c "/opt/robocomp/lib/" /etc/ld.so.conf)
    if [ $num -eq 0 ]; then
        echo "updaing the shared libraries"
        echo "/opt/robocomp/lib/" >> /etc/ld.so.conf
        sudo ldconfig
    fi
fi
