## Robocomp with FCL (The Flexible Collision Library) support

- Packages needed:

```bash
sudo apt-get install libeigen3-dev libboost-all-dev
```

- Create a software directory inside Robocomp:

```bash
cd ~/robocomp
mkdir software
```
- Install LIBCCD:

```bash
cd ~/robocomp/software
git clone https://github.com/danfis/libccd.git
cd libccd
mkdir build
cd build
cmake ..
make
sudo make install
```

- Install FCL:

```bash
cd ~/robocomp/software
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git checkout 1473632 
mkdir build
cd build
cmake ..
make
sudo make install
```

- Install LIBNABO:

```bash
cd ~/robocomp/software
git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
mkdir build
cd build
cmake ..
make
sudo make install
```

- Install LIBPOINTMATCHER:

```bash
cd ~/robocomp/software
git clone https://github.com/ethz-asl/libpointmatcher.git
cd libpointmatcher
mkdir build
cd build
cmake ..
make
sudo make install
```

- Compile Robocomp

```bash
cd ~/robocomp/build
cmake-gui ..
```

- Select checkbox FCL_SUPPORT

```bash
push configure button
push generate button
exit

make
sudo make install
```
