PKG_OK=$(dpkg-query -W --showformat='${Status}\n' docker-ce|grep "install ok installed")
echo "Checking for package docker-ce: $PKG_OK"
if [ -z "$PKG_OK" ]
then
  echo "No docker-ce. Setting up docker-ce."
  sudo apt install apt-transport-https ca-certificates curl software-properties-common
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
  sudo apt-get update
  sudo apt-get --yes install docker-ce
fi
sudo docker run -ti -w /home/robolab/ --user robolab:robolab --rm -v $(pwd)/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu18.04  bash -x robocomp_install.sh; echo $?


