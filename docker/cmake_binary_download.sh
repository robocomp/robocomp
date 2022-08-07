set -e
version=3.23
build=2
## don't modify from here
limit=3.20
os=$(echo $version $limit | awk '{if ($1 >= $2) print "linux"; else print "Linux"}')
mkdir ~/temp
cd ~/temp
curl -OL https://cmake.org/files/v$version/cmake-$version.$build-$os-x86_64.sh
sudo mkdir /opt/cmake
sudo sh cmake-$version.$build-$os-x86_64.sh --prefix=/opt/cmake --skip-license
sudo ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake