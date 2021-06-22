FROM buildpack-deps:stretch

ENV PATH $PATH:/opt/gcc-arm-none-eabi-8-2019-q3-update/bin

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
RUN tar -xf gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2 -C /opt/

