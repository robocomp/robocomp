RoboComp must be compiled from the source code. However, an installation script is provided to automate this process as much as possible. The first step is installing the script dependencies:

```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends  \
      sudo \
      curl \
      ca-certificates
```

```{warning}
Make sure you have deleted any previous versions of RoboComp from `/usr/local/bin/robocomp*` before performing an installation. Otherwise, conflicting files may (and will) cause problems in the long run!
```

To install the rest of robocomp you can use the script:

```bash
cd ~
curl -sL https://raw.githubusercontent.com/robocomp/robocomp/development/tools/install/robocomp_install.sh | bash -s
```

```{warning}
At some point, Eigen changed its include paths. As RoboComp has not been adapted to this change yet, a symlink is required to keep it working:

`sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen`
```

%{TODO: talk about FCL}

If you have followed the instructions above, you should have the files required to start using RoboComp in your system. The following tools are not required, but they make working with RoboComp much easier:

[yakuake](https://apps.kde.org/es/yakuake/)
:	Drop-down terminal emulator based on KDE Konsole technology. It makes handling tens of terminals much easier, supporting tabs and tiling layouts. Also, it can dump all that information (the _session_) to an script which can be called later to restore it.

[meld](https://meldmerge.org/)
:	Visual diff and merge tool targeted at developers. Meld helps you compare files, directories, and version controlled projects. It provides two- and three-way comparison of both files and directories, and has support for many popular version control systems.

To install them, just run:

```bash
sudo apt-get install yakuake qttools5-dev-tools qt5-assistant meld
```