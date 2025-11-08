# A sample of running ROS1 in macOS using conda

:warning: The last LTS  release of ROS1, Noetic Ninjemys, is no longer maintained or supported since May 2025. This repository is for educational purposes only. For production use, please consider directly using ROS2, which is actively maintained and supported. [Docs of ROS2](https://docs.ros.org/en/rolling/index.html).

This repository provides a sample setup for running ROS1 (Robot Operating System) on macOS using conda. ROS1 is primarily designed for Linux. According to the [wiki](https://wiki.ros.org/noetic/Installation), the officially supported platforms are Ubuntu 20.04 LTS (Focal Fossa) and Debian 10 (Buster), which are all end-of-life currently.

Archlinux support on AUR and Windows support via MSYS2 are no longer available or maintained, although mentioned on the wiki. We have noted that ROS1 is written mainly using Python and QT, which are cross-platform. After searching for solutions, we found that [Robostack](https://robostack.github.io/) provides conda packages for ROS distributions on Linux (any distribution, new and old, as long as conda is available), macOS and Windows, partly, but useable. Below is the citation for Robostack:

```bibtex
@article{FischerRAM2021,
    title={A RoboStack Tutorial: Using the Robot Operating System Alongside the Conda and Jupyter Data Science Ecosystems},
    author={Tobias Fischer and Wolf Vollprecht and Silvio Traversaro and Sean Yen and Carlos Herrero and Michael Milford},
    journal={IEEE Robotics and Automation Magazine},
    year={2021},
    doi={10.1109/MRA.2021.3128367},
}
```

There are still many courses or labs that use ROS1. This repository aims to help users set up ROS1 on multiple platforms, including macOS and Windows and other Linux distributions. To see the limitations of using ROS1 in this way, please refer to [ROS1 Noetic - Robostack](https://robostack.github.io/noetic.html).

You may refer to the `env.yaml` to see the installed packages, but in fact it is easy to setup by following the instructions below:

```powershell
# Create a new conda environment with python 3.9
conda create -n ros1 python=3.9
conda activate ros1

# now we can just setup the full installation of ROS1 Noetic desktop
conda install robostack::ros-noetic-desktop-full
```

That's all.

To run the ROS master, you can use:

```bash
catkin_make
source ./devel/setup.bash # or setup.zsh if you use zsh
```

To use Visual Studio Code with ROS1, you can install the [ROS extension](https://marketplace.visualstudio.com/items?itemName=Ranch-Hand-Robotics.rde-ros-1) provided by Ranch Hand Robotics. Then, you need to setup the settings in VS Code to source the ROS setup file automatically when opening a new terminal:

```json
{
"ros.rosSetupScript": "~/miniforge3/envs/ros1/setup.bash",
"terminal.integrated.defaultProfile.osx": "zsh"
}
```

Moreover, if the default terminal is not `bash` or `zsh`, it is possible that the setup script cannot be sourced correctly. Note that on Windows you might need Git Bash or MSYS2 or Cygwin or things like that, or use WSL directly. To fix this issue, you need to do the following steps (we use `fish` as an example here as on my machine fish is the default terminal):

```fish
bash
conda activate ros1
code .
```

Then things should work fine.
