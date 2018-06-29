#!/usr/bin/env bash

clear

# ============================================================ #
# =============     < Installing Robocomp >      ============= #
# ============================================================ #

echo "Updating system and Installing Dependencies"

sudo apt-get update
sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev pyqt4-dev-tools libboost-test-dev libboost-filesystem-dev python-libxml2 python-xmltodict libccd-dev
sudo pip install networkx
sudo apt-get install yakuake qt4-designer

echo "Installing Source from Github"

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
    read -p "Compile with Flexible Collision Library FCL? [Yy/Nn]" yn
    case $yn in
        [Yy]* ) option=1; break;;
        [Nn]* ) option=0; echo "Not FCL_SUPPORT in Robocomp"; break;;
        * ) echo "Please answer yes or no.";;
    esac
done

if [ "$option" -eq "1" ]; then
        sudo aptitude install libeigen3-dev libboost-filesystem1.58-dev libboost-test-dev libboost-program-options-dev &&

        mkdir software
        cd ~/software
        git clone https://github.com/danfis/libccd.git
        cd libccd
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        git clone https://github.com/flexible-collision-library/fcl.git
        cd fcl
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        git clone https://github.com/ethz-asl/libnabo.git
        cd libnabo
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install

        cd ~/software
        git clone https://github.com/ethz-asl/libpointmatcher.git
        cd libpointmatcher
        mkdir build
        cd build
        cmake ..
        make -j $numberCores
        sudo make install
        
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
# =============    < Testing the Simulator >     ============= #
# ============================================================ #

echo "Fetching meshes and textures. This could take a while."

cd ~/robocomp
git annex get .

echo "The simulator is going to be launched to test. For information visit RoboComp Documentation"

cd ~/robocomp/files/innermodel
rcis simpleworld.xml





