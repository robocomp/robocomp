#!/bin/bash
apt-get update
apt-get --yes install \
    vim \
    sudo
useradd -m -s /bin/bash robolab
usermod -aG sudo robolab
echo "robolab  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

