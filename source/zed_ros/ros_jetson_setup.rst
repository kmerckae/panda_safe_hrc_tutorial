.. _ROS_Jetson_Setup:

Getting Started with ROS on Jetson 
==================================

.. role:: raw-html(raw)
    :format: html

Follow the part *How to Install ROS on Jetson Xavier NX* of |stereolabs-ros-jetsonxaviernx-tutorial| 
to install ROS and to configure your catkin workspace on the Jetson Xavier NX.

In case you get 'sudo: rosdep: command not found' after 

.. code-block:: bash

    sudo rosdep init

it means that you have not yet installed rosdep. 
To install rosdep, follow |rosdep-install-tutorial|. 


.. note::
    We installed ROS Melodic. A newer ROS version is available, ROS Noetic Ninjemys, but it targets Ubuntu 20.04 Focal Fossa. 
    Ubuntu Focal does not yet officially supports CUDA and is not available on Nvidia Jetson boards at this time.

.. |stereolabs-ros-jetsonxaviernx-tutorial| raw:: html

            <a href="https://www.stereolabs.com/blog/ros-and-nvidia-jetson-xavier-nx/" target="_blank">this tutorial</a>

.. |rosdep-install-tutorial| raw:: html

            <a href="http://wiki.ros.org/rosdep" target="_blank">this tutorial</a>
