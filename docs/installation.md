## System Requirements:

- Tested on Ubuntu 22.04 with Ryzen 7 and RTX 3050

- CUDA-enabled GPU recommended; CPU-only mode supported (See notes at the end of the readme)

## 📦 Installation

### Requirements
- CMake ≥ 3.16
- C++17 compiler (GCC/Clang/MSVC)
- [optional] Graphviz (for Doxygen diagrams)
- [optional] Docker (provided dev container)

### Setup:
Assumes Cuda compatible hardware (If not then change the base image  in roboplan.Dockerfile to ubuntu22.04)
## Docker Image Installation 
1. Install the Docker **Engine** [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)

Steps to create the Docker Image **cpproboplan** (~15 min)
1. From root of the directory  run  `./scripts/build/build.sh`


## To run the docker Conatiner

Steps to create a docker container **cpproboplan:latest** that will run it in a intractive mode

1. From root of the directory run `./scripts/deploy/devel.sh`

To launch multiple shells in docker container **cpproboplan:latest**, run the following
1. run `docker exec -it mppi_docker bash` . This will attach the current terminal to the docker container mppi_docker. 


### Build

1. Navigate to `/root/workspace/src` in docker container and configure the CMakeLists.txt
2. build the project `./build.sh`