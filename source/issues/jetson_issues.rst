Jetson issues
=============




.. _jetson_shell_mode:

Set Jetson in Shell mode and download the IA model
--------------------------------------------------

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
