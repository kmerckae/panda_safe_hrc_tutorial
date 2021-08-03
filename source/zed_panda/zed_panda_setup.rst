Setup the ZED camera with Panda Arm
===================================

Ethernet connection between Panda Arm's computer and Jetson
-----------------------------------------------------------

Make a ros network between the Jetson and the panda computer
but with the IPs 192.168.4.65 for the computer
and 192.168.4.66 for the Jetson
otherwise the robot would not work.

Download and build the ROS package
----------------------------------

Clone the ROS package we made on the computer in the directory you want:

.. code-block:: bash

    cd path/to/your/directory
    git clone https://github.com/panda-brubotics/franka_constrained_control.git

Do the steps in the README file to build it.