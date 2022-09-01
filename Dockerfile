FROM ubuntu:20.04

WORKDIR /code

ARG DEBIAN_FRONTEND=noninteractive

# Install general tools: Git, Cmake, GCC, OpenMP
RUN apt-get -y update && apt-get -y install \
    build-essential \
    git-all \
    nano \
    cmake

# Install libraries used in this project
RUN apt-get -y update && apt-get -y install \
    libyaml-cpp-dev \
    libmosquitto-dev

# Install googletest
RUN apt-get -y update && apt-get -y install libgtest-dev

# Install mosquitto clients
RUN apt update -y && apt install mosquitto mosquitto-clients -y

SHELL ["/bin/bash", "-c"]
