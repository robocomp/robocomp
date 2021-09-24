#!/bin/bash
# Update Ubuntu Software repository
sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends  \
      ca-certificates \
      cmake \
      curl \
      freeglut3-dev \
      g++ \
      gcc-multilib \
      git \
      git-annex \
      libboost-dev \
      libboost-system-dev \
      libboost-thread-dev \
      libgsl-dev \
      libopenscenegraph-dev \
      libpyside2-dev \
      libqt5opengl5-dev \
      libqt5xmlpatterns5-dev \
      libzeroc-icestorm3.7 \
      make \
      python3 \
      python3-argcomplete \
      python3-distutils \
      python3-pip \
      python3-prompt-toolkit \
      python3-pyparsing \
      python3-pyside2.qtcore \
      python3-pyside2.qtwidgets \
      python3-setuptools \
      python3-termcolor \
      python3-zeroc-ice \
      sudo \
      zeroc-ice-all-dev \
      zeroc-icebox \
  && sudo rm -rf /var/lib/apt/lists/*

# Some incompatibility from pyside2 (5.15) and default qt installation (5.14)
sudo pip3 install rich typer

# pyside2-tools have removed pyside2-uic
echo "uic -g python \$@"  | sudo tee -a /usr/bin/pyside2-uic && sudo chmod a+x /usr/bin/pyside2-uic