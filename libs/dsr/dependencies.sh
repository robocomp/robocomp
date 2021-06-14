#! bin/bash 
#Please make sure that bin/bash is the location of your bash terminal,if not please replace with your local machine's bash path

sudo apt install libasio-dev
sudo apt install libtinyxml2-dev 
sudo apt install libopencv-dev
sudo apt install libqglviewer-dev-qt5
sudo apt install libeigen3-dev
sudo apt install python3-dev python3-pybind11
sudo apt install cmake gcc-10 g++-10

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 1
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 1

echo "Installing third-party software cppitertools"
sudo git clone https://github.com/ryanhaining/cppitertools /usr/local/include/cppitertools
cd /usr/local/include/cppitertools
sudo mkdir build
cd build
sudo cmake ..
sudo make install

echo "Installing third-party software Fast-RTPS"

mkdir -p ~/software
cd ~/software
git clone https://github.com/eProsima/Fast-CDR.git
mkdir Fast-CDR/build 
cd Fast-CDR/build
export MAKEFLAGS=-j$(($(grep -c ^processor /proc/cpuinfo) - 0))
cmake ..
cmake --build . 
sudo make install 
cd ~/software
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build 
cd build
cmake ..
cmake --build . 
sudo make install 
cd ~/software
git clone https://github.com/eProsima/Fast-DDS.git
mkdir Fast-DDS/build 
cd Fast-DDS/build
cmake ..
cmake --build . 
sudo make install
sudo ldconfig

cd ~/robocomp/classes/dsr/
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
