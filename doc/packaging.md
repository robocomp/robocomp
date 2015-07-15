-#Packaging Robocomp

##deb binary packages

we are using Cpack integrated with CMake for packaging robocomp.

    cd ~/robocomp
    mkdir build
    cd build
    cmake ..
    cmake-gui .. # optional
    make package
    bash fixup_deb.sh

will create a .deb package which we can install using any packaging application like dpkg.The fixup_deb.sh script will fix the control file permissions in the script. To install the created package, just double click on it(open with Software Center) or in terminal type :

    sudo dpkg -i <packagename>.deb

##Uploading package to ppa

launchpad will only accept source packages and not binary.Launchpad will then build the packages. For building source packages we are using debuild which is a wrapper around the *dpkg-buildpackage + lintian*. so you will need to install debuild and dput on your system;

The source_package.cmake script is used to create debian source package.

The main CMakeLists.txt file defines a target `spackage` that builds the source package in build/Debian with `make spackage`

For uploading the package to ppa, First change the **PPA\_PGP\_KEY** in [package_details.cmake](../cmake/package_details.cmake#L26) to details to the full-name of the PGP key  details registered with your ppa account.Then create a source package by building the target *spackage*.Once the Source package is build successfully, upload it to your ppa by:

    cd Debian/
    dput ppa:<lp-username>/<ppa-name> <packet->source.changes

building of source package can be tested with:
    
    cd Debian/robocomp-<version>
    debuild -i -us -uc -S

If you are uploading a new version of robocomp, change the version number  accordingly in the [toplevel cmake](../CMakeLists.txt#L31) before building, and then upload the source package as mentioned.

###Note:

If you want to upload another source package to ppa which doesn't have any changes in the source but maybe in the debian files. you can build the spackage after commenting out `set(DEB_SOURCE_CHANGES "CHANGED" CACHE STRING "source changed since last upload")` in [package_details.cmake](../cmake/package_details.cmake#L27) so that the the script will only increase the ppa version number and won't include the source package for uploading to ppa (which otherwise will give an error).

##Installing robocomp from ppa

First you will need to add the ppa in your sources, and then install robocomp package.

    sudo add-apt-repository ppa:<lp-username>/robocomp
    sudo apt-get update
    sudo apt-get install robocomp

this will install robocomp along with basic components into /opt/robocomp.
