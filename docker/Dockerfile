FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm

# Create non-root user
RUN useradd -m robouser && \
    apt-get update && \
    apt-get install -y sudo git && \
    echo "robouser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user
USER robouser
WORKDIR /home/robouser

# Clone and install
RUN git clone https://github.com/alfiTH/robocomp.git && \
    cd robocomp && \
    bash robocomp_install.sh -y

RUN apt-get clean && rm -rf /var/lib/apt/lists/*


CMD ["tail", "-f", "/dev/null"]
