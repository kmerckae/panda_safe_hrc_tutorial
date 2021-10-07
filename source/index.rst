.. panda robot documentation master file, created by
   sphinx-quickstart on Tue Jun  8 09:57:25 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. role:: raw-html(raw)
    :format: html

Safe Human-Robot Collaboration
===============================
This tutorial explains different ways of safe Human-Robot Collaboration (HRC),
from safe human-robot coexistence to safe physical human-robot interaction.
All the experiments are validated on the Franka Emika Panda robot.

A part of the documentation is specifically written for the use of the Panda robot in the R&MM lab
at Vrije Universiteit Brussel, but this is always mentioned on top of the page.
The major part is generally written such that researchers in other research groups can also follow this tutorial.

---

In the *Gazebo Simulation* chapter ...

.. toctree::
   :maxdepth: 1
   :caption: Gazebo Simulation

   gazebo_panda/gazebo_start



In the *Franka Emika Panda Robot Arm* chapter you will get acquainted with the Panda robot.
At the end, you will be able to control the robot via the Franka Control Interface (FCI)
and you will be able to plan a trajectory with MoveIt and follow it.

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


In the *Vicon Motion Capture* chapter you will learn how to use the Vicon motion capture data
online for the control of the Panda robot.
Add info you've sent to interns and colleagues, later on you can update it with all necessary info for your specific programs. 

.. toctree::
   :maxdepth: 1
   :caption: Vicon Motion Capture

   vicon_panda/start


In the *Constrained Control for HRC* chapter you will see how you can use the trajectory-based Explicit Reference Governor
for the real-time motion control of the Panda robot in safe human-robot coexistence scenarios
with the Vicon motion capture system.
We will explain how to set up the scenarios and which programs to run.
We refer to [1] for more detailed information about the constrained control law that is used.

.. toctree::
   :maxdepth: 1
   :caption: Constrained Control for HRC
   
   erg_panda/start


In the *ZED 2 Stereo Camera* chapter you will learn to run the ZED 2 camera, which is a stereo camera from Stereolabs, on the Nvidia Jetson Xavier NX. 
It is also possible to use a computer which has an Nvidia graphics card and Nvidia drivers. 
Note that a real-time kernel is required for the real-time control of the Panda robot 
and that Nvidia binary drivers are not supported on real-time kernels, which is why we use these small embedded computers. 
At the end you will learn how to receive the ZED 2 camera data via ROS.

.. toctree::
   :maxdepth: 1
   :caption: ZED 2 Stereo Camera

   zed_jetson/jetson_setup
   zed_jetson/zed_jetson
   zed_jetson/zed_examples
   zed_ros/ros_jetson_setup
   zed_ros/rviz_jetson
   zed_ros/rviz_external_pc
   zed_ros/object_detection
   zed_ros/octomap


In the *Constraint-Based Planning for HRC* chapter you will see how you can use a MoveIt planner, the trajectory-based Explicit Reference Governor, and the combination of the two
for the real-time motion control of the Panda robot in safe human-robot coexistence scenarios with the ZED 2 camera.
We will explain how to set up the scenarios and which programs to run.
We refer to [2] for more detailed information about the constrained control law that is used.

.. toctree::
   :maxdepth: 1
   :caption: Constraint-Based Planning for HRC

   zed_panda/start


In the *Optimization-Based Control for HRC* chapter ...

.. toctree::
   :maxdepth: 1
   :caption: Optimization-Based Control for HRC

   mpc_panda/start


In the *Physical HRI* chapter ...

.. toctree::
   :maxdepth: 1
   :caption: Physical HRI

   phri_panda/start




.. toctree::
   :maxdepth: 1
   :caption: Zed camera and Panda Arm programs

   zed_panda/zed_panda_setup
   zed_panda/zed_panda_objects_detection
   zed_panda/zed_panda_octomap

.. toctree::
   :maxdepth: 1
   :caption: Object detection

   AI/purpose
   AI/data
   AI/train

.. toctree::
   :maxdepth: 1
   :caption: Jetson, ZED2 and PANDA ARM issues

   issues/jetson_issues
   issues/jetson_zed_panda
   issues/panda_arm_issues

References
-----------
[1] K. Merckaert, B. Convens, C. Wu, A. Roncone, M. M. Nicotra, and B. Vanderborght,
**Real-time motion control of robotic manipulators for safe human-robot coexistence**,
*Robotics and Computer-Integrated Manufacturing*, vol.73, 2022, |10.1016/j.rcim.2021.102223|.



.. |10.1016/j.rcim.2021.102223| raw:: html

    <a href="https://doi.org/10.1016/j.rcim.2021.102223" target="_blank">10.1016/j.rcim.2021.102223</a>

Attribution
-----------
Major contributors to the *Constraint-Based Planning with a Stereo Camera* tutorial are listed: Kelly Merckaert, Thomas Lefevre, Binjie Dai, Kouassi Agbetoglo.
