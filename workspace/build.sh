#!/bin/bash
set -e  # Exit on error

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure only if necessary
if [ ! -f CMakeCache.txt ]; then
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
fi

# Build with verbose output
cmake --build . -- -j$(nproc) VERBOSE=1

# if build succeeds
if [ $? -eq 0 ]; then
    echo "Build succeeded."
else
    echo "Build failed!"
fi