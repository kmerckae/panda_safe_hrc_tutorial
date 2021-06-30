ZED and Jetson issues
=====================

This section will list all the problems we encountered while using the NVIDIA Jetson NX and the ZED2 camera.

NVIDIA Jetson Xavier NX boot fail
---------------------------------

The Jetson shutdown while we were using it and when we restarted it this message appeared

.. image:: images/jetson_issues_boot_failed.jpg
    :width: 600

The Jetson failed to boot and everything on the microSD Card has been erased. The first solution was to rewrite the image to the microSD Card but it also failed

.. image:: images/flash_failed.jpg
    :width: 600

If you are experimenting the same issue just use a new microSD Card. Here the `link to the thread <https://forums.developer.nvidia.com/t/nvidia-jetson-xavier-nx-boot-fail/182229?u=kouassi948>`_ we created on the NVIDIA forum.

