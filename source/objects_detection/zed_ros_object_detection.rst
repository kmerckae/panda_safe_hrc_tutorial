Objects detection with ZED2 and ROS
===================================

Download example code
---------------------

| NB: Do this in your catkin workspace
| If you have not done it yet, the example code for the zed camera running with ROS can be cloned with:

.. code-block:: bash

    git clone https://github.com/stereolabs/zed-ros-wrapper.git
    git clone https://github.com/stereolabs/zed-ros-examples.git

Add objects detection in rviz
-----------------------------
| You can find `how to add object detection in rviz in this tutorial <https://www.stereolabs.com/docs/ros/object-detection/>`_
| NB : If you launch the zed2 camera with rviz  with a new model of the object detection, it will download the AI model.
| In our case, the Jetson shutdown every time we try to download an AI model

We avoided this problem by installing the AI model with the shell mode on Jetson:

.. _jetson_shell_mode:

* Set shell mode on jetson:

.. code-block:: bash

   # To disable GUI on boot, run:
   sudo systemctl set-default multi-user.target

* While in shell mode:

    * Enter the username and password, here it is:

        * username: xavier
        * password: JetsonXavier

    * Enable wifi and ethernet connection in the text mode Network Manager:

    .. code-block:: bash

        nmtui  # open network manager

    * Run rviz:

    .. code-block:: bash

        roslaunch zed_display_rviz display_zed2.launch

    * Once the AI model downloaded, come back to graphical mode:

    .. code-block:: bash

        # To enable GUI again issue the command:
        sudo systemctl set-default graphical.target

* Now you can launch rviz with object detection on graphical mode.