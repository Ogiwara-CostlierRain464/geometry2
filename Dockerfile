# From M1
#
# docker build -t geometry2 --platform linux/amd64  .  # to run with x86_64 architecture
# docker run -it --platform linux/amd64 geometry2   # to run with x86_64 architecture
# docker run -it --platform linux/amd64 -v $(pwd):/root/ros_ws/src/geometry2 geometry2 
#
# Normal x86_64
#
# docker build -t geometry2   .  # to run with x86_64 architecture
# docker run -it  geometry2   # to run with x86_64 architecture
#
# Dev purposes, comment the source code deployment at the end and do it manually.
# Particularly:
# $ catkin_make
# $ source devel/setup.bash
# $ source src/geometry2/tf2/measure/script.bash
#
# docker run -it -v $(pwd):/root/ros_ws/src/geometry2 -v /tmp:/tmp geometry2 

FROM ros:melodic
ARG DEBIAN_FRONTEND=noninteractive

# install helpful developer tools
RUN apt-get update && apt-get install -y \
      bash-completion \
      byobu \
      ccache \
      fish \
      glances \
      tshark \
      python3-argcomplete \
      python3-pip \
      tree \
      less \
      uuid-runtime \
      htop \
      gnuplot \
      vim \
      libgflags-dev \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws/src

################  comment for dev. purposes ###############
# install dependencies
# RUN git clone https://github.com/ros/geometry2 -b melodic-devel
COPY . geometry2
WORKDIR /root/ros_ws/
RUN apt-get update && \ 
        # rosdep update && \
        rosdep install --from-paths src --ignore-src --rosdistro melodic -y
RUN . /opt/ros/melodic/setup.sh && \
        catkin_make
################

COPY ./entrypoint.sh /
RUN ["chmod", "+x", "/entrypoint.sh"]
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

