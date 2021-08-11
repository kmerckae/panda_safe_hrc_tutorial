.. panda robot documentation master file, created by
   sphinx-quickstart on Tue Jun  8 09:57:25 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Safe Human-Robot Collaboration
===============================
This tutorial explains different ways of safe Human-Robot Collaboration (HRC), 
from safe human-robot coexistence to safe physical human-robot interaction. 
All the experiments are validated on the Franka Emika Panda robot. 

A part of the documentation is specifically written for the use of the Panda robot in the R&MM lab 
at Vrije Universiteit Brussel, but this is always mentioned on top of the page. 
The major part is generally written such that researchers in other research groups can also follow this tutorial.

.. toctree::
   :maxdepth: 1
   :caption: Franka Emika Panda Robot Arm  

   panda_arm/start_desktop
   panda_arm/robot_network
   panda_arm/franka_desk
   panda_arm/robot_operating_modes
   panda_arm/FCI_project
   panda_arm/robot_control
   panda_arm/path_planning 

.. toctree::
   :maxdepth: 1
   :caption: Vicon Motion Capture with ROS Integration   

   vicon_panda/start

.. toctree:: 
   :maxdepth: 1
   :caption: Constrained Control in HRC Scenarios
   
   erg_panda/start

.. toctree::
   :maxdepth: 1
   :caption: ZED 2 Stereo Camera on NVIDIA Jetson

   zed_jetson/jetson_setup
   zed_jetson/zed_jetson
   zed_jetson/zed_examples

.. toctree::
   :maxdepth: 1
   :caption: ZED 2 Stereo Camera with ROS Integration

   zed_ros/ros_jetson_setup
   zed_ros/rviz_jetson
   zed_ros/rviz_external_pc
   zed_ros/object_detection
   zed_ros/octomap

.. toctree::
   :maxdepth: 1
   :caption: Constraint-Based Planning in HRC Scenarios

   zed_panda/start

.. toctree:: 
   :maxdepth: 1
   :caption: Constraint-Based Planning for physical HRI

   phri_panda/start

.. toctree::
   :maxdepth: 1
   :caption: ZED 2 for Panda robot

.. toctree::
   :maxdepth: 1
   :caption: Zed camera and Panda Arm programs

   zed_panda/zed_panda_setup
   zed_panda/zed_panda_objects_detection    
   zed_panda/zed_panda_octomap

.. toctree::
   :maxdepth: 1
   :caption: Jetson, ZED2 and PANDA ARM issues

   issues/jetson_issues
   issues/jetson_zed_panda
   issues/panda_arm_issues

Attribution
-----------
Major contributors to the *Constraint-Based Planning with a Stereo Camera* tutorial are listed: Kelly Merckaert, Thomas Lefevre, Binjie Dai, Kouassi Agbetoglo. 