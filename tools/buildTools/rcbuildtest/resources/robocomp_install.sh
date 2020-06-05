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
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui freeglut3-dev libboost-system-dev libboost-thread-dev qt5-default libqt5xmlpatterns5-dev libxt-dev libboost-test-dev libboost-filesystem-dev  libccd-dev  libqt5opengl5-dev
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libzeroc-ice3.7 libzeroc-icestorm3.7 python3-zeroc-ice zeroc-glacier2 zeroc-ice-slice zeroc-ice-utils zeroc-icebox zeroc-icegrid zeroc-icepatch2 zeroc-icebridge libzeroc-ice-dev zeroc-ice-all-dev zeroc-ice-compilers
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip python3-setuptools python3-pyparsing python3-numpy python3-libxml2 python3-xmltodict

sudo pip3 install networkx pyside2 argcomplete termcolor cogapp

git clone -b $ROBOCOMP_BRANCH https://github.com/robocomp/robocomp.git
sudo ln -s ~ /home/robocomp
echo "export ROBOCOMP=~/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
export ROBOCOMP=~/robocomp
export PATH=$PATH:/opt/robocomp/bin
sudo [ -d /opt/robocomp ] && sudo rm -r /opt/robocomp
cd robocomp
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
sudo ldconfig
rcnode& python3 /home/robolab/robocomp/tools/robocompdsl/autogeneration_tests/test_cdsl/test_component_generation.py --no-execution --avoid agm
