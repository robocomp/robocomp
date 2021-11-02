cat /etc/issue
branch="${branch:-development}"
source <(curl -sL https://raw.githubusercontent.com/robocomp/robocomp/$branch/tools/install/resources/robocomp_prerequisites_install.sh)

git clone -b $branch https://github.com/robocomp/robocomp.git
sudo ln -s ~ /home/robocomp
echo "export ROBOCOMP=~/robocomp" >> ~/.bashrc
echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
echo "export PYTHONIOENCODING=utf-8" >> ~/.bashrc
export ROBOCOMP=~/robocomp
export PATH=$PATH:/opt/robocomp/bin
export PYTHONIOENCODING=utf-8
sudo [ -d /opt/robocomp ] && sudo rm -r /opt/robocomp
cd robocomp
sudo pip3 install tools/cli/
rcconfig init
mkdir build
cd build
cmake ..
make -j$(nproc --ignore=2)
sudo env "PATH=$PATH" PYTHONIOENCODING=utf-8 make install
sudo sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
sudo ldconfig
