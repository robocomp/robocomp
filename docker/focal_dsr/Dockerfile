FROM robocomp/robocomp:focal_dsr_dev
# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal for production"
ARG DEBIAN_FRONTEND=noninteractive
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
  libtinyxml2-6a \
  && rm -rf /var/lib/apt/lists/*

RUN ls -lah /home/robocomp/
COPY --from=robocomp/robocomp:focal_dsr_dev /opt/robocomp/ /opt/robocomp/
#COPY --from=robocomp/robocomp:focal_dsr_dev /home/robolab/robocomp/components/dsr-graph/components/ /home/robolab/robocomp/components/dsr-graph/components/
#COPY --from=robocomp/robocomp:focal_dsr_dev /home/robolab/robocomp/components/dsr-graph/robots_pyrep/ /home/robolab/components/dsr-graph/robots_pyrep/
#COPY --from=robocomp/robocomp:focal_dsr_dev /home/robolab/robocomp/components/dsr-graph/etc/autonomyLab_complete.simscene.json /home/robolab/robocomp/components/dsr-graph/etc/autonomyLab_complete.simscene.json
COPY --from=robocomp/robocomp:focal_dsr_dev /usr/local/ /usr/local/
COPY --from=robocomp/robocomp:focal_dsr_dev /usr/lib/python3/dist-packages/ /usr/lib/python3/dist-packages/ 
RUN ldconfig
USER robolab
WORKDIR /home/robolab/robocomp/components/

# docker build -t robocomp/robocomp:focal_dsr .
# docker-compose up    

