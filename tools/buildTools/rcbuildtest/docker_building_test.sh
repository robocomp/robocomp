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
if [ -z "$1" ] && [ "$1" != '18.04' ] && [ "$1" != '19.04' ] && [ "$1" != '16.04' ]
then
    echo "Currently you only can use 16.04, 18.04 or 19.04 as Ubuntu versions"
    UBUNTU_VERSION='18.04'
else
    UBUNTU_VERSION=$1
fi

if [ -z "$2" ] || [ "$2" != 'stable' ] && [ "$2" != 'development' ]
then
    echo "Currently you only can use stable or development as robocomp branches."
    ROBOCOMP_BRANCH='stable'
else
    echo "WHY"
    ROBOCOMP_BRANCH=$2
fi


sudo docker run --name robocomp_test -it -w /home/robolab/ --user robolab:robolab -v $(pwd)/../../install/robocomp_install.sh:/home/robolab/robocomp_install.sh robocomp/clean-testing:robocomp-ubuntu$UBUNTU_VERSION bash -l -x robocomp_install.sh $ROBOCOMP_BRANCH
if [ $? == 0 ]; then
  echo "built done"
else
  #sudo docker logs robocomp_test | mail -s "docker" elqueseaelmail@gmail.com
fi
sudo docker container rm robocomp_test


