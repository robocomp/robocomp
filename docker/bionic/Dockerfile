#Download base image ubuntu 18.04
FROM ubuntu:bionic

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Bionic"

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update Ubuntu Software repository
RUN apt update && apt install -y curl sudo

# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/robolab && \
    echo "robolab:x:${uid}:${gid}:Robolab,,,:/home/robolab:/bin/bash" >> /etc/passwd && \
    echo "robolab:x:${uid}:" >> /etc/group && \
    echo "robolab:robolab"|chpasswd && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab-user && \
    chown ${uid}:${gid} -R /home/robolab 

RUN cd /home && sudo ln -s ~ robocomp

USER robolab
#Define the ENV variable
ENV ROBOCOMP=~/robocomp
ENV PATH=$PATH:/opt/robocomp/bin
WORKDIR /home/robolab/
# Install Robocomp
RUN curl -sL https://raw.githubusercontent.com/robocomp/robocomp/development/tools/install/robocomp_install.sh | bash -

