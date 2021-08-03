.. _ZED_ROS_Object_Detection:

Object Detection on External Computer
=======================================

.. role:: raw-html(raw)
    :format: html

Here we explain how you can visualize the detected objects on your external computer. 

Requirements
------------

* You have to be able to :ref:`run rviz on your external computer<Rviz_External_PC>`.   

* :ref:`Clone the franka_constrained_control project <FCI_Project>` or |make_new_catkin-ws| on your external computer. 

* Download required files on your external computer
  
  * :raw-html:`<font color="red">  Why not clone zed-interfaces and zed-ros-examples in catkin_ws/src on external computer??   </font>`
  
  * Download the following zip file on your external computer

        :download:`zed_packages_for_external_computer <doc/zed_packages_for_external_computer.zip>` 

  * Unzip the files and drag them in your ``catlin_ws/src`` directory
  * Go to your catkin_ws file and build it, this will build the plugin for display bunding box around detected objects and the zed libraries

    .. code-block:: bash
        
        cd ~/catkin_ws
        catkin_make
  
.. |make_new_catkin-ws| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment" target="_blank">make a new catkin workspace</a>

Visualize the detected objects in rviz
----------------------------------------

First enable object detection on the Jetson. 

* Go to the zed wrapper configuration directory on the Jetson and open the zed2.yaml file. 

  .. code-block:: bash

      cd path/to/catkin_ws/src/zed-ros-wrapper/zed_wrapper/params
      gedit zed2.yaml

* At line 20: set ``od_enabled`` to *true* to enable object detection

* At line 21: set ``model`` to *1* to detect all kind of objects that are available in the library

* You can also change other |stereolabs-object-detection-parameters-explained| 

Once the ROS network is established, vizualize the detected objects in rviz. 

* Open a new terminal on the **external computer** and run

  .. code-block:: bash

      roscore

* Open a new terminal on the **Jetson** and run 

  .. code-block:: bash

      roslaunch zed_display_rviz display_zed2.launch

* Open another terminal on the **external computer** and run

  .. code-block:: bash

      rosrun rviz rviz

To display the bounding boxes around the detected objects, you have to add ``ZedOdDisplay`` which you can find under *rviz_plugin_zed_od*
and select the */zed2/zed_node/obj_det/objects* topic. 

To have a better idea of the detected objects, you can display the point cloud to the display. Therefore you have to add ``PointCloud2`` 
and select the */zed2/zed_node/point_cloud/cloud_registered* topic. 

Finally you should see something like this:

.. image:: images/object_detection_rviz.png
    :align: center
    :width: 700px


.. |stereolabs-object-detection-parameters-explained| raw:: html

    <a href="https://www.stereolabs.com/docs/ros/zed-node/#object-detection-parameters-only-zed-2-and-zed-2i" target="_blank">object detection parameters</a>
