#!/usr/bin/env bash

clear

# Colors
white="\033[1;37m"
grey="\033[0;37m"
purple="\033[0;35m"
red="\033[1;31m"
green="\033[1;32m"
yellow="\033[1;33m"
Purple="\033[0;35m"
Cyan="\033[0;36m"
Cafe="\033[0;33m"
Fiuscha="\033[0;35m"
blue="\033[1;34m"
transparent="\e[0m"

echo -e $white"
▄▄▄▄▄▄              ▄▄                                                        
██▀▀▀▀██            ██                                                        
██    ██   ▄████▄   ██▄███▄    ▄████▄    ▄█████▄   ▄████▄   ████▄██▄  ██▄███▄ 
███████   ██▀  ▀██  ██▀  ▀██  ██▀  ▀██  ██▀    ▀  ██▀  ▀██  ██ ██ ██  ██▀  ▀██
██  ▀██▄  ██    ██  ██    ██  ██    ██  ██        ██    ██  ██ ██ ██  ██    ██
██    ██  ▀██▄▄██▀  ███▄▄██▀  ▀██▄▄██▀  ▀██▄▄▄▄█  ▀██▄▄██▀  ██ ██ ██  ███▄▄██▀
▀▀    ▀▀▀   ▀▀▀▀    ▀▀ ▀▀▀      ▀▀▀▀      ▀▀▀▀▀     ▀▀▀▀    ▀▀ ▀▀ ▀▀  ██ ▀▀▀  
                                                                      ██      

                                                                " 

echo -e $blue"Welcome to the Installation Script for Robocomp!
"                     

echo -e $white"If you have any problems with the process, please contact us at robocomp.team@gmail.com
"

sleep 3

# Check if --help flag is provided

if [ ${#@} -ne 0 ] && [ "${@#"--help"}" = "" ]; then
  printf -- 'For more information, please see the repository\n';
  exit 0;
fi;

# ============================================================ #
# =============     < Installing Robocomp >      ============= #
# ============================================================ #

printf "Updating system and Installing Dependencies\nDepending on your internet connection, this might take a while."

sudo apt-get update
sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev pyqt4-dev-tools libboost-test-dev libboost-filesystem-dev python-libxml2 python-xmltodict libccd-dev
sudo pip install networkx
sudo apt-get install yakuake qt4-designer

printf "Installing Source from Github\n"

git clone https://github.com/robocomp/robocomp.git
cd ~/robocomp
git annex get .

echo "Adding Robocomo to $PATH"

sudo ln -s /home/$USER /home/robocomp
echo "export ROBOCOMP=/home/$USER/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
sudo echo "/opt/robocomp/lib" >> nano /etc/ld.so.conf
source ~/.bashrc
sudo ldconfig

# ============================================================ #
# =============         < FCL Support >          ============= #
# ============================================================ #

while true; do
    read -p "Compile with Flexible Collision Library FCL? [Yy/Nn]
    " yn
    case $yn in
        [Yy]* ) option=1; break;;
        [Nn]* ) option=0; echo "No FCL_SUPPORT for Robocomp. The instalation process will keep going."; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

if [ "$option" -eq "1" ]; then
        
        printf "Installing dependencies\n" 
        sudo aptitude install libeigen3-dev libboost-filesystem1.58-dev libboost-test-dev libboost-program-options-dev &&

        printf "Creating new directory for the components.\n"
        mkdir software
        cd ~/software
        printf "Downloading libccd.\n"
        git clone https://github.com/danfis/libccd.git
        cd libccd
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        printf "Downloading FCL.\n"
        git clone https://github.com/flexible-collision-library/fcl.git
        cd fcl
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        printf "Downloading libnano.\n" 
        git clone https://github.com/ethz-asl/libnabo.git
        cd libnabo
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        printf "Downloading libpointmatcher.\n"
        git clone https://github.com/ethz-asl/libpointmatcher.git
        cd libpointmatcher
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install
        
        printf "Installing FCL support.\n\n"
        cd ~/robocomp/build
        cmake-gui ..
        make -j $numberCores
        sudo make install
fi

# ============================================================ #
# =============   < Components Installation >    ============= #
# ============================================================ #

while true; do
    read -p "Install some basic components from RoboLab? [Yy/Nn]" yn
    case $yn in
        [Yy]* ) option=1; break;;
        [Nn]* ) option=0; echo "Not installing components."; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

if [ "$option" -eq "1" ]; then
    cd ~/robocomp/components
    git clone https://github.com/robocomp/robocomp-robolab.git
    echo "Components installed in ~/robocomp/components/robocomp-robolab/components"
fi

# ============================================================ #
# =============   <  Jockstick or Keyboard  >    ============= #
# ============================================================ #

while true; do
    read -p "Are you using a Joystick? [Yy/Nn]" yn
    case $yn in
        [Yy]* ) option=1; break;;
        [Nn]* ) option=0; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

if [ "$option" -eq "1" ]; then
    echo "We are installing the Joystick component."
    cd ~/robocomp/components/robocomp-robolab/components/joystickComp
    cmake .
    make
    cd bin
    sudo addgroup your-user dialout   # To solve some permissions issues in Ubuntu
    printf "Installation complete. Launching component in the background.\n"
    ./startJoyStick.sh

if [ "$option" -eq "0" ]; then    
    echo "We are installing the Keyboard component.
    You will be able to move the robot around with the arrow buttons."
    cd ~/robocomp/components/robocomp-robolab/components/keyboardrobotcontroller
    cmake .
    make
    src/keyboardrobotcontroller.py --Ice.Config=etc/config
fi


# ============================================================ #
# =============    < Testing the Simulator >     ============= #
# ============================================================ #

echo "Fetching meshes and textures. This could take a while."

cd ~/robocomp
git annex get .

echo "The simulator is going to be launched to test if it works.
"
echo "For more information visit the repository and help."

printf "Installation complete. Have a great time using Robocomp!\n"
cd ~/robocomp/files/innermodel
rcis simpleworld.xml





