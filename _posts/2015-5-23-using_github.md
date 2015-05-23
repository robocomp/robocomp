---
layout: post
title: Maintaining your own repository of components in GitHub 
categories: [Tutorial]
tags: [github]
description: We recommend that you create a repository for your components (i.e. *mycomponents* directory in the example before) in your GitHub account (or other similar site) and pull/clone it in *~/robocomp/components* whenever yo need it....
---

#Maintaining your own repository of components in GitHub

We recommend that you create a repository for your components (i.e. *mycomponents* directory in the example before) in your GitHub account (or other similar site) and pull/clone it in *~/robocomp/components* whenever yo need it. For example, if your GitHub account is *myaccount*, first log in with your browser and create a new repository named *mycomponents* following this instructions: 

    https://help.github.com/articles/create-a-repo/
    
Now is good time to write down a short description of what your component does in the README.md file.

Then we need to clean up the binary and generated files in *myfirstcomp*. Note that this is not necessary if you upload the component to the repo just after creating it with DSLEditor and before you type *cmake .*

    cd ~/robocomp/components/mycomponents/myfirstcomp
    make clean
    sudo rm -r CMakeFiles
    rm CMakeCache.txt
    rm cmake_install.cmake
    rm Makefile
    rm *.kd*
    rm src/moc*
    sudo rm -r src/CMakeFiles
    rm src/cmake_install.cmake
    rm src/Makefile
    
now we are ready:

    cd ~/robocomp/components/mycomponents
    git init
    git remote add origin "https://github.com/myaccount/mycomponents.git"
    git add mycomponents
    git push -u origin master
    
You can go now to GitHub and chek that your sources are there!
    
  
