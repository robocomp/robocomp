#Download base image ubuntu 18.04
FROM ubuntu:focal

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal"

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt update && apt install -y firefox sudo

# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/robolab && \
    chmod 0440 /etc/sudoers.d/robolab && \
    chown ${uid}:${gid} -R /home/robolab


    
# Install Robocomp
RUN apt-get install --no-install-recommends --yes git git-annex cmake make g++ libgsl-dev libopenscenegraph-dev cmake-qt-gui freeglut3-dev libboost-system-dev libboost-thread-dev qt5-default libqt5xmlpatterns5-dev libxt-dev libboost-test-dev libboost-filesystem-dev  libccd-dev  libqt5opengl5-dev libxml2-dev
RUN apt-get install --no-install-recommends --yes libzeroc-ice3.7 libzeroc-icestorm3.7 python3-dbus python3-zeroc-ice zeroc-glacier2 zeroc-ice-slice zeroc-ice-utils zeroc-icebox zeroc-icegrid zeroc-icepatch2 zeroc-icebridge libzeroc-ice-dev zeroc-ice-all-dev zeroc-ice-compilers
RUN apt-get install --no-install-recommends --yes python3-pip python3-setuptools python3-pyparsing python3-numpy python3-libxml2 python3-xmltodict
RUN apt-get install --no-install-recommends --yes libfcl-dev
RUN pip3 install networkx pyside2 argcomplete termcolor cogapp prompt_toolkit

USER robolab
WORKDIR /home/robolab
ENV HOME /home/robolab
#Define the ENV variable
ENV ROBOCOMP=~/robocomp
ENV PATH=$PATH:/opt/robocomp/bin

RUN git clone https://github.com/robocomp/robocomp.git
RUN echo "export ROBOCOMP=/home/robolab/robocomp" >> ~/.bashrc
RUN echo "export PATH=$PATH:/opt/robocomp/bin" >> ~/.bashrc
RUN export ROBOCOMP=/home/robolab/robocomp
RUN export PATH=$PATH:/opt/robocomp/bin
RUN cd /home/robolab/robocomp && mkdir build && cd build && cmake -DFCL_SUPPORT=True .. && make -j$(nproc)


USER root
RUN cd /home/robolab/robocomp/build && make install
RUN sh -c "echo '/opt/robocomp/lib/' >> /etc/ld.so.conf"
RUN ldconfig
RUN sudo ln -s ~ /home/robocomp

