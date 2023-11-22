=================
Archive
=================

.. role:: raw-html(raw)
    :format: html


.. note:: The following things should be restructered when updating the whole tutorial. 


-----------------------------
Octomap on External Computer
-----------------------------
.. _ZED_ROS_Octomap:


.. role:: raw-html(raw)
    :format: html
    
This page will help you to make and save an Octomap on your external computer. 

Requirements
------------

* You have to be able to :ref:`run rviz on your external computer<Rviz_External_PC>`.  

* :ref:`Clone the franka_constrained_control project <FCI_Project>` or |make_new_catkin-ws-2| on your external computer. 

* Install the ROS Octomap packages on your external computer

  .. code-block:: bash

      sudo apt-get update
      sudo apt-get install ros-melodic-octomap ros-melodic-octomap-server ros-melodic-octomap-mapping ros-melodic-octomap-ros ros-melodic-octomap-msgs

* Create a package for the Octomap on the external computer

  * :raw-html:`<font color="red">  Why not clone zed-interfaces and zed-ros-examples in catkin_ws/src on external computer??   </font>`
  
  * Download the folowing zip file:

        :download:`octomap_tools package <doc/octomap_tools.zip>` 

  * Unzip and drag the files in your ``catkin_ws/src`` directory

  * build your catkin workspace

    .. code-block:: bash

      cd ~/catkin_ws
      catkin_make

.. |make_new_catkin-ws-2| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment" target="_blank">make a new catkin workspace</a>

Vizualize the Octomap in rviz
-------------------------------

You can first modify the Octomap parameters to your requirements. 

* Go to the launch directory in the octomap_tools package, which you can find in 

  .. code-block:: bash

      cd path/to/catkin_ws/src/octomap_tools/launch

* In the file ``save_octomap_from_pointcloud.launch`` you can modify the |ros-octomap-parameters| that are explained in Section 2.2.4.

.. |ros-octomap-parameters| raw:: html

    <a href="http://wiki.ros.org/octomap_server" target="_blank">Octomap parameters</a>
 
Once the ROS network is established, vizualize the Octomap in rviz. 

* Open a new terminal on the **external computer** and run

  .. code-block:: bash

      roscore

* Open a new terminal on the **Jetson** and run 

  .. code-block:: bash

      roslaunch zed_wrapper zed2.launch

* Open another terminal on the **external computer** and run

  .. code-block:: bash

      cd path/to/catkin_ws
      source devel/setup.sh
      roslaunch octomap_tools save_octomap_from_pointcloud.launch

Finally you should see something like this:

.. image:: images/save_octomap.png
    :align: center
    :width: 700px

Save an Octomap and visualize an existing Octomap
-------------------------------------------------

You can save the Octomap vizualized in rviz while rviz is still running. 
Therefore you have to go to the directory where you want to save the Octomap and run the *octomap_saver* server with the name you want to give to the Octomap. 

.. code:: bash

  cd path/to/octomap-directory/
  rosrun octomap_server octomap_saver -f <octomap-name>.bt

To vizualize a saved Octomap, you have to run on your external computer

.. code-block:: bash

    roslaunch octomap_tools load_octomap.launch path:=path/to/octomap-directory/<octomap-name>.bt rviz_octomap:=true



--------------------------------------
Setup the ZED camera with Panda Arm
--------------------------------------

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




-----------------------------------------------------------
Objects detection with ZED2 in Panda arm environnement
-----------------------------------------------------------

What does this program ?
------------------------

| This pogram uses the zed camera's object detection module, you can set the object you want to detect in the launch file.
| Once the camera detects an object it will track it and the robot can plan a trajectory to the object.
| If the object is lost, the camera will look for another same type object.

Modify parameters in the launch file
------------------------------------

* Go to the zed_franka_planner package, in the launch directory open the file ``grab_detected_object_test.launch`` with your favorite editor
* From here you can change some parameters
* The parameter ``object_to_be_detected`` can be `one of the listed here <https://www.stereolabs.com/docs/api/group__Object__group.html#ga13b0c230bc8fee5bbaaaa57a45fa1177>`_


Launch the program
------------------

* On the computer run ``roscore``
* On the jetson run ``roslaunch zed_wrapper zed2.launch``
* On the computer and in another shell source the workspace and launch the program

  .. code-block:: bash
    
    cd path/to/your/folder/franka_constrained_control/catkin_ws
    source devel/setup.sh
    roslaunch zed_franka_planner grab_detected_object_test.launch

* Once rviz is launched, load the parameters saved in the file ``path/to/the/package/zed_franka_planner/rviz/zed_panda_object_tracking.rviz``


Modify the camera transformation parameters
-------------------------------------------

* The panda robot's axis are:

.. image:: images/panda_axis.png
    :width: 300

* The map frame's origin is the position of the camera when you launch the zed_wrapper node on the jetson and its axis are:

.. image:: images/zed_map_axis.png
    :width: 300

* To modify the transformation between the camera and the robot:

  * Go to rviz

  * From the display ``Static Transform Publisher`` you can set the map and robot frames and the transformation parameters:
    
    .. .. image:: images/agni_tf_tools.png
   
-----------------------
Octomap for Panda arm
-----------------------

This section will help you to make and save an octomap of the panda arm environnement

Parameters for the octomap
--------------------------

Set parameters to filter the pointcloud
***************************************

In this section we will set the parameters used to filter the point cloud so we can create an octomap from the filtered point cloud
In the filterd point cloud, the robot is removed

* When you are in your ``catkin_ws`` directory go to ``src/franka_ros/panda_moveit_config/config`` 
* Open the file ``sensors_kinect_pointcloud.yaml``
* From this file, you can change some parameters (source : `Perception Pipeline Tutorial <http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/perception_pipeline/perception_pipeline_tutorial.html>`_):

  * The general parameters are:
  
    * *sensor_plugin:* The name of the plugin that we are using.
    * *max_update_rate:* The octomap representation will be updated at rate less than or equal to this value.

  * Parameters specific to the Point cloud updater are:
  
    * *point_cloud_topic:* This specifies the topic to listen on for a point cloud.
    * *max_range:* (in m) Points further than this will not be used.
    * *point_subsample:* Choose one of every point_subsample points.
    * *padding_offset:* The size of the padding (in cm).
    * *padding_scale:* The scale of the padding.
    * *filtered_cloud_topic:* The topic on which the filtered cloud will be published (mainly for debugging). The filtering cloud is the resultant cloud after self-filtering has been performed.



Set the octomap parameters:
***************************
  
* Go to the octomap_tools package
* Go to the launch directory
* Open the file ``save_panda_octomap.launch``
* From this file you can change some parameters (lines 22 to 25)
* The parameters are listed and explained `here in the section 2.2.4 <http://wiki.ros.org/octomap_server>`_

Create and save the octomap
---------------------------

* Go to your catkin_ws source your workspace and run the program

.. code-block:: bash

  cd path/to/your/workspace
  source devel/setup.sh
  roslaunch octomap_tools save_panda_octomap.launch

* In rviz load the config localized in the package ``zed_franka_planner`` in rviz/zed_panda_save_octomap.rviz
* Adjust the transformation between the camera and the robot by using the ``Static Transform Publisher`` in rviz
* Write down the transformation in quaternion
* Once you get a satisfying octomap, you can save it:

  * Go to the directory where you want to save the octomap
  * Open a terminal there
  * Run ``rosrun octomap_server octomap_saver -f panda_robot_octomap.bt``

Plan in the octomap environnement
---------------------------------
| Set your work space like this: 

.. image:: images/plan_in_octomap_workspace.jpg
  :width: 600

| Save an octomap of your workspace
| This program is a demo of planning in the octomap

* Go to your catkin_ws source your workspace and run the program

.. code-block:: bash

  cd path/to/your/workspace
  source devel/setup.sh
  roslaunch zed_franka_planner plan_in_octomap.launch path:=path/to/your/saved/octomap panda_to_map_transfo:="1.3 0.06 0.38 0 0 1 0"
  # "1.3 0.06 0.38 0 0 1 0" is the transformation in quaternion between the camera and the robot when you saved the octomap

* In rviz load the config ``plan_in_octomap.rviz`` (same directory as earlier)




----------------------------------------------
Training a Neural Network for Object Detection
----------------------------------------------