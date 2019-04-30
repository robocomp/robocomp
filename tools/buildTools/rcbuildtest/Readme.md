
RoboComp Build Test
===============================

This script let you test build of the Robocomp uploaded code on git with different Ubuntu versions (16.04, 18.04, 19.04).
You just need to execute the script:
```bash
sh docker_building_test.sh [ubuntu version number] [Robocomp branch name]
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
