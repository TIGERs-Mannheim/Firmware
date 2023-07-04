FROM ubuntu:20.04

ENV PATH $PATH:/opt/gcc-arm-none-eabi-8-2019-q3-update/bin

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update

RUN apt install -y cmake git make wget bzip2 curl g++ && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
RUN tar -xf gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2 -C /opt/
