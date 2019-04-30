
RoboComp Build Test
===============================

This script lets you test build of Robocomp uploaded code on git with different Ubuntu versions (16.04, 18.04, 19.04).
You just need to execute the script:
```bash
sh docker_building_test.sh [-v=ubuntu version number] [-b=Robocomp branch name]
```
Without parameters this script try to compile the stable Robocomp branch on Ubuntu 18.04.
You can specify other Ubuntu versions with `-v=` or `--version=` options. Currently only 16.04, 18.04 and 19.04 versions are supported)
```bash
sh docker_building_test.sh -v=19.04
```
You can also specify other branch with the `-b=` or `--branch=` option. Currently only stable and development are supported)
```bash
sh docker_building_test.sh -b=development
```

## To Do  
* Check the available ubuntu versions listed as tags of https://hub.docker.com/r/robocomp/clean-testing/tags and been able to use any of those.  
* Check the available branches listed by git for Robocomp and been able to use any of those.  
