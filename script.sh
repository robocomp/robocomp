#!/bin/bash
# @Author: doody
# @Date:   2016-07-21 12:30:59
# @Last Modified by:   swapsharma
# @Last Modified time: 2016-08-24 23:51:02
# @Todo: Support for all major os
# @Todo: Need to make script more robust

# To install git-annex
echo "Cloning git-annex repository ..."
git clone https://github.com/joeyh/git-annex.git ~/git-annex

echo "Installing stack ..."
# Install stack
os=`uname -s`
distro=`lsb_release -i | cut -f2`

case ${os} in
	"Linux" )
		# Installing haskell-platform
		sudo apt-get install haskell-platform

		# Installing stack
		case ${distro} in
			"Ubuntu" )
				sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 575159689BEFB442
				version=`lsb_release -r | cut -f2`
				case ${version} in
					16.04 ) echo 'deb http://download.fpcomplete.com/ubuntu xenial main'|sudo tee /etc/apt/sources.list.d/fpco.list
						;;
					15.10 ) echo 'deb http://download.fpcomplete.com/ubuntu wily main'|sudo tee /etc/apt/sources.list.d/fpco.list
						;;
					14.04 ) echo 'deb http://download.fpcomplete.com/ubuntu trusty main'|sudo tee /etc/apt/sources.list.d/fpco.list
						;;
					12.04 ) echo 'deb http://download.fpcomplete.com/ubuntu precise main'|sudo tee /etc/apt/sources.list.d/fpco.list
						;;
				esac

				sudo apt-get update && sudo apt-get install stack -y
				;;
			"Debian" )
				sudo apt-get install haskell-stack zlib1g-dev libtinfo-dev
				;;
		esac
		;;
	"Darwin" )
		brew install ghc
		brew install haskell-stack
		;;
esac

echo "Compiling git-annex using stack ..."
cd ~/git-annex
stack setup
stack install
echo "Copied fom ~/.local/bin/git-annex to /bin"
sudo mv ~/.local/bin/git-annex /bin

# To set-up rclone
echo "Downloading rclone ..."
cd ~/
case ${os} in
	"Linux" )
		arch=`uname -i`
		case ${arch} in
			x86_64 ) wget http://downloads.rclone.org/rclone-current-linux-386.zip
				;;
		esac
		;;
	"Darwin" )
		arch=`uname -p`
		case ${arch} in
			i386 ) curl "http://downloads.rclone.org/rclone-current-osx-386.zip" -o rclone-v1.17-linux-amd64.zip
			;;
		esac
		;;
esac

echo "Installing rclone ..."
case ${os} in
	"Linux" )
		unzip rclone-v1.17-linux-amd64.zip
		cd rclone-v1.17-linux-amd64
		;;
	"Darwin" )
		unzip rclone-v1.17-linux-amd64.zip
		cd rclone-v1.32-osx-386
		;;
esac

#copy binary file
sudo cp rclone /usr/sbin/

case ${os} in
	"Linux" )
		sudo chown root:root /usr/sbin/rclone
		;;
	"Darwin" )
		sudo chown root:admin /usr/sbin/rclone
		;;
esac

sudo chmod 755 /usr/sbin/rclone
#install manpage
sudo mkdir -p /usr/local/share/man/man1
sudo cp rclone.1 /usr/local/share/man/man1/

case ${os} in
	"Linux" )
		sudo mandb
		;;
	"Darwin" )
		sudo /usr/libexec/makewhatis
		;;
esac

echo "Downloading and Installing git-annex-remote-rclone ..."
# To set up git-annex-remote-rclone
cd ~
case ${os} in
	"Linux" )
		wget https://github.com/DanielDent/git-annex-remote-rclone/archive/master.zip
		;;
	"Darwin" )
		curl "https://codeload.github.com/DanielDent/git-annex-remote-rclone/zip/master" -o git-annex-remote-rclone-master.zip
		;;
esac

unzip git-annex-remote-rclone-master.zip
cd git-annex-remote-rclone-master
sudo cp git-annex-remote-rclone /usr/sbin

case ${os} in
	"Linux" )
		sudo chown root:root /usr/sbin/git-annex-remote-rclone
		;;
	"Darwin" )
		sudo chown root:admin /usr/sbin/git-annex-remote-rclone
		;;
esac

sudo chmod 755 /usr/sbin/git-annex-remote-rclone

# Steps to make git annex usage abstract
echo "
tool_init() {
	git annex init "My_Laptop"
	echo "$@"
	git annex initremote "$@"
}

tool_add() {
	git annex add $1
	git commit -m "Remote files added" $1
	git push -u origin master
	git annex sync --content
	git annex copy $1 --to $2
}

tool_get() {
	git annex sync
	git annex enableremote $2
	git annex get $1 --from $2
}

tool_opt() {
	case \$1 in
		initremote ) tool_init "${@:2}"
			;;
		add ) tool_add "${@:2}"
			;;
		get ) tool_get "${@:2}"
			;;
	esac
}

alias git_tool=tool_opt
" >> ~/.bash_aliases
source ~/.bash_aliases