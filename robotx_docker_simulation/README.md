# robotx_docker_simulation

## simulation tools for distributed environment using Nvidia docker and docker-compose

### dependency install
1. install [Nvidia docker](https://github.com/NVIDIA/nvidia-docker)
1. install dokcer compose

```
sudo apt update
sudo apt install docker-compose  
```

### hardware requirement
Nvidia GPU (this programs was tested on [GTX 1060 3GB](https://www.kuroutoshikou.com/product/graphics_bord/nvidia/gf-gtx1060-3gb_oc_df/))

### how to run
Scripts in this package does not contains ros API.
So, you don't have to run roscore before run simulation.

1. build docker image (just first time)
```
rosrun robotx_docker_simulation build.py  
```
1. bringup docker images
```
rosrun robotx_docker_simulation bringup.py  
```
