Objects detection with ZED2 in Panda arm environnement
======================================================

Install ZED libraries and plugin
--------------------------------

The needed libraries and rviz plugin are already added in our package,
If you want to do it in your own package,
follow the steps in :ref:`object detection with ros<obj_detection_on_panda_computer>`

Modify the camera transformation parameters
-------------------------------------------

* The launch file for the object detection with panda is in the package ``zed_franka_planner``
* Open the file grab_detected_object_test.launch with your editor
* How to determine the transformation parameters:

    * You can `convert rotation angle to quaternion on this website <https://quaternions.online/>`_
    * The panda robot's axis are:

    .. image:: images/panda_axis.png

    * The map frame's origin is the position of the camera when you launch the zed_wrapper node on the jetson and its axis are:

    .. image:: images/zed_map_axis.png

* Modify the transformation parameter ``transfo_world2map`` directly in the launch file by changing its default value or when you do the roslaunch for instance you can launch it this way : ``roslaunch zed_franka_planner grab_detected_object_test.launch transfo_world2map:="0 0 0 0 0 0 1"``
    
    * "0 0 0 0 0 0 1" are the transformation parameters the 3 first numbers are for the translation and the 4 next are for the rotation in quaternion.

Launch the program
------------------

* On the computer run ``roscore``
* On the jetson run ``roslaunch zed_wrapper zed2.launch``
* On the computer and in another shell source de workspace and run ``roslaunch zed_franka_planner grab_detected_object_test.launch``
