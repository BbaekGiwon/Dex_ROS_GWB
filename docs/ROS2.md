# Installation 

**Contributor:** Chanyoung Ahn <br>
**Reference:** [IsaacSim 5.0 Docs](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html)

## Environment
* ROS2 requires `python 3.10`, but IsaacLab+ROS2 (Especially, sim-5.0.0) require `python 3.11`. To build environment, we use Isaac+ROS2 Docker image. **However,** we should use commercial version of docker, I developed the enviroment from docker image to `apptainer/singularity`. This approach is more adaquate for using HPC/server, I stronglly recommand use singularity (If you also want to use KISTI HPC, just use singluarity). 

### Singularity
* Installation Link: [Link](https://docs.sylabs.io/guides/3.5/user-guide/introduction.html)