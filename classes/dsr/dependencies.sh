      mkdir ~/software
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
