# Introduction to debian packaging

All debain packages should follow certain conventions. The root source directory should contain a directory named *debian*. This directory contains files which stores info about the package.
These are the required files under the debian directory

* __rules__
This is the maintainer script for the package building. This script is run by the packaging application to build and install the source into a *tmp* directory in the debian folder. It has the following Targets

  * *clean target* : to clean all compiled, generated, and useless files in the build-tree.

  * *build target* : to build the source into compiled programs and formatted documents in the build-tree.

  * *build-arch target* : to build the source into arch-dependent compiled programs in the build-tree.

  * *build-indep target* : to build the source into arch-independent formatted documents in the build-tree.

  * *binary target* : to create all binary packages (effectively a combination of binary-arch and binary-indep targets)

  * *binary-arch target* : to create arch-dependent (Architecture: any) binary packages in the parent directory.

  * *binary-indep target*: to create arch-independent (Architecture: all) binary packages in the parent directory.

* __changelog__
This file contains the project changelog along with the project name , version and distribution and urgency of your package.

* __compact__
The compact file defines the debhelper compatibility level.

* __debian/control__
This file contains various values which dpkg, dselect, apt-get, apt-cache, aptitude, and other package management tools will use to manage the package. The control file describes the source and binary package, and gives some information about them, such as their names, who the package maintainer is, build and run dependencies and so on.

* __copyright__
This file contains information about the copyright and license of the upstream sources

* __(pre/post)(inst/rm)__
This are the scipts which are run before or after installation or removal of package.

Now once you have the source directory in the prescribed format. you will need a *.tar.gz* archive of the source in the same folder.Then we can create a debian binary package using

    debuild -i -us -uc -b

Or a debian source package using

    debuild -i -us -uc -S

