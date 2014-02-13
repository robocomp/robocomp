Uncompress ReflexxesTypeII.zip

sudo cp ReflexxesTypeII/include/*.h /usr/local/include
cd ReflexxesTypeII/Linux
make clean64 all64
cd ../..
sudo cp ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so /usr/local/lib

Finally, include these two lines in CMakeListSpecific.txt: 

SET ( LIBS ${LIBS} -lReflexxesTypeII )
ADD_DEFINITIONS( -I/usr/local/include/Reflexxes )

