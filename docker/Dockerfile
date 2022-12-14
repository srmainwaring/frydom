# ==========================================================================
# FRyDoM - frydom-ce.org
#
# Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
# All rights reserved.
#
# Use of this source code is governed by a GPLv3 license that can be found
# in the LICENSE file of hydro.
#
# ==========================================================================

FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y -q apt-utils dialog
RUN apt-get install -y -q build-essential
RUN apt-get install -y -q xserver-xorg-dev
RUN apt-get install -y -q x11proto-xf86vidmode-dev
RUN apt-get install -y -q libxxf86vm-dev
RUN apt-get install -y -q mesa-common-dev
RUN apt-get install -y -q libgl1-mesa-dev
RUN apt-get install -y -q libglu1-mesa-dev
RUN apt-get install -y -q libxext-dev
RUN apt-get install -y -q libxcursor-dev
RUN apt-get install -y -q freeglut3-dev
RUN apt-get install -y -q libcurl4-gnutls-dev
RUN apt-get install -y -q python3-sphinx
RUN apt-get install -y -q python3-sphinx-rtd-theme
RUN apt-get install -y -q doxygen
RUN apt-get install -y -q wget
RUN apt-get install -y -q awscli

RUN apt-get install -y -q cmake
RUN apt-get install -y -q git
RUN apt-get install -y -q graphviz
RUN apt-get install -y -q libirrlicht-dev
RUN apt-get install -y -q libirrlicht1.8


# https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive
RUN apt-get install -y -q tzdata
# RUN DEBIAN_FRONTEND=noninteractive apt-get install -y -q tzdata

ARG download_dir=/download

# Building VTK
RUN mkdir ${download_dir} && \
    cd ${download_dir} && \
    wget --no-verbose https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz && \
    tar xf VTK-8.2.0.tar.gz && \
    cd VTK-8.2.0 && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DVTK_WRAP_PYTHON=OFF && \
    make -j6 && \
    make install && \
    cd / && \
    rm -rf ${download_dir}

# Update links to shared objects
RUN /sbin/ldconfig -v
