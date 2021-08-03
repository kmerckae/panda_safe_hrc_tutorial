.. _ZED_ROS_Octomap:

Octomap on External Computer
============================

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