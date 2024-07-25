FROM ubuntu:22.04

ENV PATH $PATH:/opt/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update

RUN apt install -y cmake git make wget bzip2 curl g++ xz-utils && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz
RUN tar -xf arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz -C /opt/
