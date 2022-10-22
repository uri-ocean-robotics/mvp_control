Installation
============

Target system
-------------

ROS-MVP is only suitable for Ubuntu 20.04 and ROS Noetic as of now.
It requires 64-bit processor.
It has been tested on intel CPUs and Raspberry Pi 4 varians (CM4 and SBC).

Installation from source
------------------------

.. warning::
    MVP is only available through source installation.

Pull the ``mvp_msgs`` repository if you don't have it already

.. code-block:: bash

    git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_msgs

Pull the ``mvp_control`` repository

.. code-block:: bash

    git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_control

Install dependencies

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

.. note::
    To execute the command above you need to be root level of your workspace.

And compile using **catkin**.