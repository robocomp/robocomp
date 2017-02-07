##Robocomp with FCL (The Flexible Collision Library) support
```
sudo aptitude install libeigen3-dev libboost-filesystem1.55-dev libboost-test-dev libboost-program-options-dev
```
- Install LIBCCD:
```
cd ~/software
git clone https://github.com/danfis/libccd.git
cd libccd
mkdir build
cd build
cmake ..
make
sudo make install
```
- FCL:
```
cd ~/software
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 1473632 
mkdir build
cd build
cmake ..
make
sudo make install
```
- LIBNABO:
```
cd ~/software
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
mkdir build
cd build
cmake ..
make
sudo make install
```
- LIBPOINTMATCHER:
```
cd ~/software
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build
cd build
cmake ..
make
sudo make install
```
- Compiling Robocomp
```
cd ~/robocomp/build
cmake-gui ..
```
select checkbox FCL_SUPPORT
```
push configure button
push generate button
exit

make
sudo make install
```
