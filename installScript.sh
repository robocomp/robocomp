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
cyan="\033[0;36m"
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

echo -e $cyan"Updating system and Installing Dependencies\nDepending on your internet connection, this might take a while.
"
echo -e $white

sudo apt-get update
sudo apt-get install git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice-all-dev freeglut3-dev libboost-system-dev libboost-thread-dev qt5-default libqt5xmlpatterns5-dev libqt5opengl5-dev libxt-dev libboost-test-dev libboost-filesystem-dev  libccd-dev zeroc-ice-all-runtime
sudo apt-get install python3-pip python3-setuptools python3-pyparsing=2.2.0+dfsg1-2 python3-numpy python3-libxml2 python3-xmltodict python3-zeroc-ice
sudo pip3 install networkx pyside2 argcomplete termcolor cogapp
sudo apt-get install yakuake qttools5-dev-tools qt5-assistant

echo ""
echo -e $cyan"Installing Source from Github.
"
echo -e $white

git clone --recursive https://github.com/robocomp/robocomp.git
cd ~/robocomp

echo -e $cyan"Preparing folder and synchronizing files."

echo -e $white

git annex get .

echo -e $cyan"Adding Robocomp to $PATH."

sudo ln -s /home/$USER /home/robocomp
echo "export ROBOCOMP=/home/$USER/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
sudo echo "/opt/robocomp/lib" >> nano /etc/ld.so.conf

echo -e $cyan"Caching new libraries."

source ~/.bashrc
sudo ldconfig

# ============================================================ #
# =============         < FCL Support >          ============= #
# ============================================================ #

echo -e $white

while true; do
    read -p "Compile with Flexible Collision Library (FCL)? [Yy/Nn]
    " yn
    case $yn in
        [Yy]* ) option=1; break;;
        [Nn]* ) option=0; echo "No FCL_SUPPORT for Robocomp. The instalation process will keep going."; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

if [ "$option" -eq "1" ]; then
        
        echo -e $cyan"Installing dependencies." 
        sudo apt-get install aptitude
        sudo aptitude install libeigen3-dev libboost-filesystem1.58-dev libboost-test-dev libboost-program-options-dev &&

        echo -e $cyan"Creating new directory for the components."
        mkdir software
        cd ~/software
        echo -e $cyan"Downloading libccd."
        git clone https://github.com/danfis/libccd.git
        cd libccd
        mkdir build
        cd build
        echo -e $cyan"Building libccd."
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        echo -e $cyan"Downloading FCL."
        git clone https://github.com/flexible-collision-library/fcl.git
        cd fcl
        mkdir build
        cd build
        echo -e $cyan"Building FCL." 
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        echo -e $cyan"Downloading libnano." 
        git clone https://github.com/ethz-asl/libnabo.git
        cd libnabo
        mkdir build
        cd build
        echo -e $cyan"Building libnano." 
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        echo -e $cyan"Downloading libpointmatcher."
        git clone https://github.com/ethz-asl/libpointmatcher.git
        cd libpointmatcher
        mkdir build
        cd build
        echo -e $cyan"Building libpointmatcher.\n"
        cmake ..
        make -j $numberCores
        sudo make install
        
        echo -e $cyan"Finishing Installation of FCL support.\n\n"
        cd ~/robocomp/build
        cmake-gui ..
        make -j $numberCores
        sudo make install
        echo -e $yellow"Installation finished."
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
    echo "Installing components. This might take a while."
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
    echo -e $cyan"Building..."
    make
    cd bin
    sudo addgroup your-user dialout   # To solve some permissions issues in Ubuntu
    echo -e $white"Installation complete. Launching component in the background."
    ./startJoyStick.sh

else [ "$option" -eq "0" ];   
    echo "We are installing the Keyboard component.
    You will be able to move the robot around with the arrow buttons."
    cd ~/robocomp/components/robocomp-robolab/components/keyboardrobotcontroller
    echo -e $white"Building..."
    cmake .
    make
    echo "Making sure you will be able to control the robot..."
    echo -e $red"If keyboard doesn't move the robot, please visit repository for more info."
    src/keyboardrobotcontroller.py --Ice.Config=etc/config
fi


# ============================================================ #
# =============    < Testing the Simulator >     ============= #
# ============================================================ #

echo -e $white"Fetching meshes and textures. This might take a while."

cd ~/robocomp
git annex get .
echo -e $yellow"Installation of textures finished."

echo "The simulator is going to be launched to test if it works."
echo "For more information visit the repository."

echo -e $cyan"Installation complete. Have a great time using Robocomp!"
cd ~/robocomp/files/innermodel
echo -e $white"Lauching Robocomp."
rcis simpleworld.xml

exit



