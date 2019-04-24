PKG_OK=$(dpkg-query -W --showformat='${Status}\n' docker-ce|grep "install ok installed")
echo "Checking for package: $PKG_OK"
if [ -z "$PKG_OK" ]
then
  echo "No somelib. Setting up somelib."
  sudo apt-get --force-yes --yes install the.package.name
fi
