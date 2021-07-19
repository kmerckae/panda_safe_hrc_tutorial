Setup the ZED camera with Panda Arm
===================================

Ethernet connection between Panda Arm's computer and Jetson
-----------------------------------------------------------

Do the steps in :ref:`ethernet connection <ethernet_connection>`
but with the IPs 192.168.4.65 for the computer
and 192.168.4.66 for the Jetson
otherwise the robot would not work.

Download and build the ROS package
----------------------------------

Clone the ROS package we made:

.. code-block:: bash

    git clone https://github.com/panda-brubotics/franka_constrained_control.git

Do the steps in 

Install ZED libraries and plugin
--------------------------------

