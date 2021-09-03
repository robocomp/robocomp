#!/bin/bash
echo "sudo apt-get update"
sudo apt-get update
echo "sudo apt-get --yes install apt-transport-https ca-certificates curl gnupg-agent software-properties-common"
sudo apt-get --yes install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
echo "curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -"
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
echo "sudo add-apt-repository \"deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable\""
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
echo "sudo apt-get update"
sudo apt-get update
echo "sudo apt-get --yes install docker-ce docker-ce-cli containerd.io"
sudo apt-get --yes install docker-ce docker-ce-cli containerd.io
echo "sudo groupadd docker"
sudo groupadd docker
echo "sudo gpasswd -a $USER docker"
sudo gpasswd -a $USER docker
echo "newgrp docker"
newgrp docker