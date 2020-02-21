cat /etc/issue
if [ -z "$1" ] || [ "$1" != 'stable' ] && [ "$1" != 'development' ]
then
    echo "Currently you only can use stable or development as robocomp branches."
    ROBOCOMP_BRANCH='stable'
else
    ROBOCOMP_BRANCH=$1
fi

sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice-all-dev freeglut3-dev libboost-system-dev libboost-thread-dev qt5-default libqt5xmlpatterns5-dev libxt-dev libboost-test-dev libboost-filesystem-dev  libccd-dev zeroc-ice-all-runtime libqt5opengl5-dev
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip python3-setuptools python3-pyparsing=2.2.0+dfsg1-2 python3-numpy python3-libxml2 python3-xmltodict python3-zeroc-ice
#sudo DEBIAN_FRONTEND=noninteractive apt-get install -y git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui zeroc-ice-all-dev freeglut3-dev libboost-system-dev libboost-thread-dev qt4-dev-tools python-pip  python-pyparsing python-numpy python-pyside pyside-tools libxt-dev libboost-test-dev libboost-filesystem-dev python-libxml2 python-xmltodict libccd-dev python-zeroc-ice zeroc-ice-all-runtime
sudo pip3 install networkx pyside2 argcomplete termcolor cogapp

git clone -b $ROBOCOMP_BRANCH https://github.com/robocomp/robocomp.git
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
sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
sudo ldconfig
rcportchecker ports
