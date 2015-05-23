---
layout: post
title: Chroot environment
categories: [Post]
tags: [Chroot environment]
description: A chroot is a way of isolating applications from the rest of your computer, by putting them in a jail. This is particularly useful if you are testing an application which could potentially alter important system files, or which may be insecure....
---

#Chroot environment

A chroot is a way of isolating applications from the rest of your computer, by putting them in a jail. This is particularly useful if you are testing an application which could potentially alter important system files, or which may be insecure.
A chroot is basically a special directory on your computer which prevents applications, if run from inside that directory, from accessing files outside the directory. In many ways, a chroot is like installing another operating system inside your existing operating system. 
The following are some possible uses of chroots:

1. Isolating insecure and unstable applications
2. Running 32-bit applications on 64-bit systems
3. Testing new packages before installing them on the production system
4. Running older versions of applications on more modern versions of Ubuntu
5. Building new packages, allowing careful control over the dependency packages which are installed 

This manual will follow the steps specified in the [official page of Ubuntu](https://help.ubuntu.com/community/BasicChroot). And the system we will install as tutorial is Ubuntu 14.04 Trusty amd64.

##Brief Explanation
Imagine you have your Robocomp version well installed and working really fine in your system (i.e. Ubuntu 14.04 amd64), but you need to upgrade your ICE or OpenCV or PCL or whatever third-party library to a new version. You don't want to risk your well functional version of Robocomp and it's dependencies removing the current version and installing the new one (this usually affects other packages and libraries), and you don't have time enough to make a whole fresh installation in other partition or virtual machine, so the fastest solution is to create a jail containing the same distribution of your main system (Ubuntu 14.04 amd64) with chroot and test Robocomp with the new version of the library you need without touching your fine Robocomp installation.
Realize that creating a chrooted environment in your machine makes your system believe that your root directory ("/") is in another place than the actual root of the system (like I explain on the wiki, the process in which you launch chroot believes that the root directory is in / while actually it is in /var/chroot/trusty_x64/, not letting you touch anything outside that directory and therefore not risking your current installation).
Another practical use for chroot is to test an especific program or library in a different distribution or architecture. For example, if you are working in Ubuntu 14.04 amd64 and you want to test if a library that you are using works fine in Debian Wheezy or Ubuntu 14.10 or Ubuntu 14.04 i386.

##Creating a chroot

1. First of all we need to install the tools to make a chroot in out system.

    `sudo apt-get install debootstrap schroot`

2. Create a folder where the chroot is going to be installed. You need to make the folder using administrator permission (with _sudo_ i.e). We will put the chroot up in _/var/chroot/trusty_x64_

    `sudo mkdir /var/chroot && sudo mkdir /var/chroot/trusty_x64`

3. Create a configuration file for schroot. For our example, we will create a file named trusty_x64.conf in _/etc/schroot/chroot.d/_

    `sudo gedit /etc/schroot/chroot.d/trusty_x64.conf`

    And write the following inside: (Change the <USERNAME> to actual username, example "root-users=abhi"

    ```
    [trusty_x64]
    description=Ubuntu trusty 14.04 for amd64
    directory=/var/chroot/trusty_x64
    root-users=<USERNAME>
    type=directory
    users=testuser
    ```
  - The first line is the name of the chroot thatis going to be created.
  - **description** is a short description of the chroot.
  - **directory** the path where the chroot is going to be installed. Note that is the same path that we specified in step 2.
  - **root-users** list of users that are allowed in our chroot without password.
  - **type**  The type of the chroot. Valid types are ‘plain’, ‘directory’, ‘file’, ‘block-device’ and ‘lvm-snapshot’. If empty or omitted, the default type is ‘plain’.
  - **users** list of users that are allowed access to the chroot.


    see [schroot.config](http://manpages.ubuntu.com/manpages/hardy/man5/schroot.conf.5.html) for further information.

4. Run Debootstrap. This step will download and unpack a basic ubuntu or debian system to the chroot directory we created in step 2.

    `sudo debootstrap --variant=buildd --arch amd64 trusty /var/chroot/trusty_x64 http://archive.ubuntu.com/ubuntu`

    In our example, we are creating a chroot of an Ubuntu 14.04 64-bit distribution, but this command allows some different commands that can satisfy our needs, for instance, if we want to install the same distribution but the 32-bit version, we have to type:

    `sudo debootstrap --variant=buildd --arch i386 trusty /var/chroot/trusty http://archive.ubuntu.com/ubuntu`

    Note that we have to do the proper changes creating a different schroot configuration file (i.e. _/etc/schroot/chroot.d/trusty_) and a different folder for the new chroot (i.e. _/var/chroot/trusty_)

    If we want to create a chroot for a Debian version (i.e. Debian Wheezy (stable)) we have to type:

    `sudo debootstrap --variant=buildd --arch amd64 wheezy /var/chroot/wheezy_x64 http://ftp.debian.org/debian`

5. Checking the chroot. To be sure that everything went ok, we can type the following command, that will list all the available chroot enviroments in out system.

    `schroot -l`

    If trusty_x64 appears, we can start working in our chrooted environment typing:

    `schroot -c trusty_x64 -u root`

    The prompt of the chrooted environment should be like:

    `(trusty_x64)root@abhi-Inspiron-7520:/home/abhi#`

    **NOTE** This step is not mandatory.
    **NOTE** For convenience, the default schroot configuration rebinds the /home directory on the host system so that it appears in the chroot system. This could be unexpected because it means that you can accidentally delete or otherwise damage things in /home on the host system. To change this behaviour we can run the following command in the host system:
    
    `sudo gedit /etc/schroot/default/fstab`

    And comment the /home line:

    ```
# fstab: static file system information for chroots.
# Note that the mount point will be prefixed by the chroot path
# (CHROOT_PATH)
#
# <file system> <mount point>   <type>  <options>       <dump>  <pass>
/proc           /proc           none    rw,bind        0       0
/sys            /sys            none    rw,bind        0       0
/dev            /dev            none    rw,bind         0       0
/dev/pts        /dev/pts        none    rw,bind         0       0
#/home          /home           none    rw,bind         0       0
/tmp            /tmp            none    rw,bind         0       0
```

And that's it! Now we have a whole very basic system in which we can test out programs and libraries. 

##Troubleshooting

* If you get locale warnings in the chroot like **"Locale not supported by C library."** or **"perl: warning: Setting locale failed."**, then try one or more of these commands:

```
    sudo dpkg-reconfigure locales
    sudo apt-get install language-pack-en
    sudo locale-gen en_US.UTF-8
    sudo dpkg-reconfigure locales
```
  if the problem persist check out this [page](http://perlgeek.de/en/article/set-up-a-clean-utf8-environment).

* To get access to the intertet within the chroot, you have to type:

    `sudo cp /etc/resolv.conf /var/chroot/trusty_x64/etc/resolv.conf`

* You might want to have the proper sources.list in order to be able to install packages from Ubuntu official repositories like universe or multiverse, and the security updates. If you make a chroot installation, the sources.list will be the most basic one, like:

    `deb http://archive.ubuntu.com/ubuntu trusty main`

  You can generate a more complete sources.list file in this pages [Ubuntu](http://repogen.simplylinux.ch/
) and [Debian](http://debgen.simplylinux.ch/)

##External Links

  [Ubuntu official chroot manual](https://help.ubuntu.com/community/BasicChroot)

  [Ubuntu official deboostrap manual](https://help.ubuntu.com/community/DebootstrapChroot)

  [PerlGeek troubleshooting](http://perlgeek.de/en/article/set-up-a-clean-utf8-environment)

  [Schroot conf manual](http://manpages.ubuntu.com/manpages/hardy/man5/schroot.conf.5.html)

  [Debootstap manual](http://manpages.ubuntu.com/manpages/trusty/en/man8/debootstrap.8.html)

  [Sources.list for Ubuntu](http://repogen.simplylinux.ch/)

  [Sources.list for Debian](http://debgen.simplylinux.ch/)


