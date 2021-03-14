# Robocomp on Docker

> If you don't know what Docker is or how to use it we recommend you to start here:  
> https://www.youtube.com/watch?v=YFl2mCHdv24
> https://blog.octo.com/en/docker-registry-first-steps/
> https://docker-curriculum.com/

In Robocomp we have created several docker files that can be used to build different images with Ubuntu versions of Robocomp.
In the subdirectories of this folder, you can find files to create Robocomp images with or without FCL on Ubuntu Bionic or Focal.
> Note: you can also find a version of Robocomp with AGM [here](https://github.com/ljmanso/AGM/tree/current/docker/bionic).

To create the Docker images, you can enter the corresponding subdirectory and run
```docker
docker build -t <image_name> .
```
## Images in Docker HUB
It is not absolutely necessary to build these images from the Dockerfiles since we have versions of the same ones uploaded to Docker Hub. The available Robocomp images can be found [here](https://hub.docker.com/r/robocomp/robocomp/tags?page=1&ordering=last_updated).

## Use cases
The uses of these images can be very varied.

### Local testing
Sometimes we use these to make local compilation tests of the last commits of Robocomp in the different versions of Ubuntu. It's like some kind of local CI, but with the continuous part. If the previously shown build command shown above ends correctly we can assume that the Robocomp installation is still working. 

### Check installation process 
We can also modify the Dockerfiles before doing the build to test the current installation process and see if there are any missing or spare packages, versions, etc. Sometimes to look into the Dockerfile is easy than reading any installation instructions.

### Developing within a Container
You can use a Docker Image as a clean environment to test your development. With the `-v` option of docker, you can mount any folder in your host machine (your current operating system) into the image you are going to run. So, if you have a new component that you can test on one of the Robocomp images you can just run the image with that option:
```docker
docker run -v<local_path_to_component>:<destination_path_in_container> -it robocomp:focal bash
```
And with that you get a bash shell inside the docker running container and you can just cd to the <destination_tah_in_container> to run or test your component.

### Tools based in Docker
We have also developed a tool on top of docker that you could be interested in: [rcbuildvalidator](https://github.com/robocomp/robocomp/tree/development/tools/buildTools/rcbuildvalidator).


### Other Dockerfiles related with Robocomp
These other Dockerfiles, dockercompose files, and related images in the docker hub could be of your interest if you are working with these projects.
#### AGM
https://github.com/ljmanso/AGM/tree/current/docker/bionic

#### DSR
https://github.com/robocomp/dsr-graph/tree/development/docker/focal


<!--stackedit_data:
eyJoaXN0b3J5IjpbLTEzODI0NTUxNTksNDI5OTg2OTEwXX0=
-->
