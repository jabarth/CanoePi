#!/bin/bash
set -e

# This script is executed inside the 'pico-builder' container.
# It assumes the PICO_SDK_PATH is set in the container environment.

echo "Starting firmware build..."

# Create a build directory if it doesn't exist
mkdir -p build
cd build

# Configure the project with CMake
# The toolchain file is part of the Pico SDK
cmake -DCMAKE_BUILD_TYPE=Release ..

# Compile the project
make

echo "Firmware build complete. The .uf2 file is in the build directory."
