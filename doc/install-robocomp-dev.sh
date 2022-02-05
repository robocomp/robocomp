#!/bin/sh


sudo apt-get update && sudo apt-get upgrade
sudo apt-get install git git-annex cmake g++ libgsl0-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice35 freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools yakuake openjdk-7-jre python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev pyqt4-dev-tools qt4-designer aptitude
sudo aptitude update #&& sudo aptitude upgrade



git clone --recursive https://github.com/robocomp/robocomp.git
cd ~/robocomp
git annex get .


sudo ln -s /home/$USER /home/robocomp
echo "export ROBOCOMP=/home/$USER/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
sudo echo "/opt/robocomp/lib" >> nano /etc/ld.so.conf
source ~/.bashrc
sudo ldconfig

numberCores=`nproc`
echo "Installing Robocomp"
sudo rm -r /opt/robocomp
cd ~/robocomp
mkdir build
cd build
cmake ..
make -j $numberCores
sudo make install

while true; do
    read -p "Compile with Flexible Collision Library FLC? [Yy/Nn]" yn
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

while true; do
    read -p "Installing components? [robolab, ursus, ursus-rockin,  all, None]" yn
    case $yn in
        "robolab" ) cd ~/robocomp/components; rm -rf robocomp-robolab;  git clone https://github.com/robocomp/robocomp-robolab.git; break;;
	"ursus" ) cd ~/robocomp/components; rm -rf robocomp-ursus;  git clone https://github.com/robocomp/robocomp-ursus.git; break;;
        "ursus-rockin" ) cd ~/robocomp/components; rm -rf robocomp-ursus-rockin; git clone https://github.com/robocomp/robocomp-ursus-rockin.git; cd ~/robocomp/components/robocomp/robocomp-ursus-rockin/files/; sh fetchSimulationModel.sh; cd makeMeCoffe; sh fetchCoffeeFiles.sh; break;;
        "all" ) cd ~/robocomp/components; rm -rf robocomp-robolab; rm -rf robocomp-ursus; rm -rf robocomp-ursus-rockin; git clone https://github.com/robocomp/robocomp-robolab.git; git clone https://github.com/robocomp/robocomp-ursus.git; git clone https://github.com/robocomp/robocomp-ursus-rockin.git; cd ~/robocomp/components/robocomp-ursus-rockin/files/; sh fetchSimulationModel.sh; cd makeMeCoffee; sh fetchCoffeeFiles.sh; break;;
        "None" ) echo "not install compontes"; break;;
        * ) echo "Incorrect Option";;
    esac
done

while true; do
    read -p "Install Active Grammar Modelling? [Yy/Nn]" yn
    case $yn in
        [Yy]* ) sudo apt-get install python-pyparsing python-pyside pyside-tools libpython2.7-dev python-dev libboost-all-dev cmake python-imaging libxml2-dev && cd ~/robocomp; git clone https://github.com/ljmanso/AGM.git; cd ~/robocomp/AGM; sh compile.sh; break;;
        [Nn]* ) echo "not install AGM"; break;; 
        * ) echo "Please answer yes or no.";;
    esac
done

echo "Install finish"
