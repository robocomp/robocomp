PKG_OK=$(dpkg-query -W --showformat='${Status}\n' docker-ce|grep "install ok installed")
echo "Checking for package docker-ce: $PKG_OK"
if [ -z "$PKG_OK" ]
then
  echo "No docker-ce. Setting up somelib."
  sudo apt-get --force-yes --yes install the.package.name
fi
sudo docker run -ti -w /home/robolab/ --user robolab:robolab --rm -v $(pwd)/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu18.04  bash -x robocomp_install.sh; echo $?


