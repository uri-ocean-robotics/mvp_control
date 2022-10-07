# MVP - Low Level Controller

This repository contains low-level controller for marine vehicles.

## Installation

Pull the `mvp_msgs` repository if you don't have it already
```bash
git clone https://github.com/GSO-soslab/mvp_mission
```

Pull the repository
```bash
git clone https://github.com/GSO-soslab/mvp_control
```

Install dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

Compile using `catkin_make`.

## Funding
This work is supported by the [National Science Foundation](https://www.nsf.gov/) award #2154901