#!/bin/bash
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
sudo docker stop robocomp_test
sudo docker kill robocomp_test
sudo docker container rm robocomp_test


ROBOCOMP_BRANCH="stable"
UBUNTU_VERSION="18.04"

for i in "$@"
do
case $i in
    -b=*|--branch=*)
    ROBOCOMP_BRANCH="${i#*=}"
    shift # past argument=value
    ;;
    -v=*|--version=*)
    UBUNTU_VERSION="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done


echo "UBUNTU VERSION  = ${UBUNTU_VERSION}"
echo "ROBOCOMP BRANCH = ${ROBOCOMP_BRANCH}"


sudo docker run --name robocomp_test -it -w /home/robolab/ --user robolab:robolab -v $(pwd)/../../install/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu$UBUNTU_VERSION bash -l -x robocomp_install.sh $ROBOCOMP_BRANCH
if [ $? = 0 ]
then
  echo "built done"
  sudo docker logs robocomp_test > last_log.log
else
  echo "built failed. Sending email"
  #sudo docker logs robocomp_test | mail -s "docker" elqueseaelmail@gmail.com
fi
sudo docker logs robocomp_test > last_log.log
sudo docker container rm robocomp_test


