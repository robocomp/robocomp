cat /etc/issue

source robocomp_prerequisites_install.sh

ROBOCOMP_BRANCH="${ROBOCOMP_BRANCH:-development}"
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
sudo env "PATH=$PATH" make install
sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
sudo ldconfig
