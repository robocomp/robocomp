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
      libqt5opengl5-dev \
      libqt5xmlpatterns5-dev \
      libzeroc-icestorm3.7 \
      make \
      python3 \
      python3-apt \
      python3-argcomplete \
      python3-distutils \
      python3-distutils-extra \
      python3-pip \
      python3-prompt-toolkit \
      python3-pyparsing \
      python3-setuptools \
      python3-termcolor \
      python3-zeroc-ice \
      software-properties-common \
      sudo \
      zeroc-ice-all-dev \
      zeroc-icebox \
  && sudo rm -rf /var/lib/apt/lists/*

sudo pip3 install rich typer pyside2
