#!/usr/bin/env bash

# This script needs to be run as admin (sudo)
# Make sure you installed headers for openGL on your system (freeglu3-dev on Ubuntu by eg)

echo "Making a temporary directory build_vtk for the VTK build"
mkdir build_vtk
cd build_vtk
echo "Downloading VTK archive"
wget --no-verbose https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz
echo "Extracting VTK archive"
tar xf VTK-8.2.0.tar.gz
cd VTK-8.2.0
mkdir build
cd build
echo "Running cmake"
cmake .. -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DVTK_WRAP_PYTHON=OFF
echo "Building VTK lib"
make -j6
echo "Installing VTK lib"
make install
echo "Removing the temporary directory"
cd ../../..
rm -rf build_vtk
ldconfig
