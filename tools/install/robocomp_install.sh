sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y git git-annex cmake g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice-all-dev freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev libboost-test-dev libboost-filesystem-dev python-libxml2 python-xmltodict libccd-dev python-zeroc-ice zeroc-ice-all-runtime
sudo pip install networkx
alias python=python2
git clone https://github.com/robocomp/robocomp.git
sudo ln -s ~ /home/robocomp
echo "export ROBOCOMP=~/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
export ROBOCOMP=~/robocomp
export PATH=$PATH:/opt/robocomp/bin
sudo [ -d /opt/robocomp ] && rm -r /opt/robocomp
cd robocomp
mkdir build
cd build
cmake ..
make
sudo make install
echo "/opt/robocomp/lib/" >> /etc/ld.so.conf
sudo ldconfig
rcportchecker ports
