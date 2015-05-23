---
layout: post
title: Introduction to debian packaging
categories: [Post]
tags: [Debian]
description: For packaging a program there should be a subdirectory under the program's source directory, called debian. Files in this directory customize the behavior of the...
---

#Introduction to debian packaging

For packaging a program there should be a subdirectory under the program's source directory, called debian. Files in this directory customize the behavior of the package. The most important of them are control, changelog, copyright, and rules, which are required for all packages.

* **debian/control**  
   This file contains various values which dpkg, dselect, apt-get, apt-cache, aptitude, and other package management tools will use to manage the package. The control file describes the source and binary package, and gives some information about them, such as their names, who the package maintainer is, build and run dependencies and so on.

* **debian/changelog**  
   This is the log of changes to the Debian package.This is a required file, which has a special format described in [Debian Policy Manual](https://www.debian.org/doc/debian-policy/ch-source.html#s-dpkgchangelog). This format is used by dpkg and other programs to obtain the version number, revision, distribution, and urgency of your package.

* **debian/rules**  
This is the maintainer script for the package building. This script is run by the packaging application to build and install the source into a *tmp* directory in the debian folder. It has the following Targets

  * *clean target* : to clean all compiled, generated, and useless files in the build-tree.

  * *build target* : to build the source into compiled programs and formatted documents in the build-tree.

  * *build-arch target* : to build the source into arch-dependent compiled programs in the build-tree.

  * *build-indep target* : to build the source into arch-independent formatted documents in the build-tree.

  * *binary target* : to create all binary packages (effectively a combination of binary-arch and binary-indep targets)

  * *binary-arch target* : to create arch-dependent (Architecture: any) binary packages in the parent directory.

  * *binary-indep target*: to create arch-independent (Architecture: all) binary packages in the parent directory.

* **debian/(post/pre-inst/rm)**  
These files are executable scripts which are automatically run before or after a package is installed. Along with a file named control, all of these files are part of the "control" section of a Debian archive file.

