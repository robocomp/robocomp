cat /etc/issue

source <(curl -sL https://raw.githubusercontent.com/robocomp/robocomp/development/tools/install/resources/robocomp_prerequisites_install.sh)

ROBOCOMP_BRANCH="${ROBOCOMP_BRANCH:-development}"
git clone -b $ROBOCOMP_BRANCH https://github.com/robocomp/robocomp.git
sudo ln -s ~ /home/robocomp
echo "export ROBOCOMP=~/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
echo "export PYTHONIOENCODING=utf-8" >> ~/.bashrc
export ROBOCOMP=~/robocomp
export PATH=$PATH:/opt/robocomp/bin
export PYTHONIOENCODING=utf-8
sudo [ -d /opt/robocomp ] && sudo rm -r /opt/robocomp
cd robocomp
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo env "PATH=$PATH" PYTHONIOENCODING=utf-8 make install
sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
sudo ldconfig
