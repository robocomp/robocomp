FROM ubuntu:22.04

RUN apt-get update && apt-get install -y git
RUN git clone https://github.com/alfiTH/robocomp.git
RUN cd robocomp && bash robocomp_install.sh -y

CMD ["tail", "-f", "/dev/null"]
