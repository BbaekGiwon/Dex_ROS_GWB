# Installation 

**Contributor:** Chanyoung Ahn <br>
**Reference:** [IsaacSim 5.0 Docs](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html)

## Environment
* ROS2 requires `python 3.10`, but IsaacLab+ROS2 (Especially, sim-5.0.0) require `python 3.11`. To build environment, we use Isaac+ROS2 Docker image. **However,** we should use commercial version of docker, I developed the enviroment from docker image to `apptainer/singularity`. This approach is more adaquate for using HPC/server, I stronglly recommand use singularity (If you also want to use KISTI HPC, just use singluarity). 

### Singularity
* Installation Link: [Link](https://docs.sylabs.io/guides/4.3/user-guide/quick_start.html)
* Install Go lang (Just follow upper link). 


### IsaacLab Container Deployment
The following is written based on the official docs.
Please refer to [Isaac Lab Container Deployment](https://isaac-sim.github.io/IsaacLab/main/source/deployment/cluster.html) Section. You can skip the Apptainer / Singularity installation part as we are using unprivileged binaries. While following the instructions above, please note the modified instructions below for our system.

#### Install Apptainer
```shell
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

### Build ROS2+IsaacSim 


### docker2Singularity Image
```shell
./docker/cluster/cluster_interface.sh push [profile]
```

