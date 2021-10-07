.. _Path_Planning:

Plan and Follow a Trajectory with MoveIt
=========================================

.. role:: raw-html(raw)
    :format: html

.. note:: This page is generally written. 
          For the *Panda 2* robot in the R&MM lab at Vrije Universiteit Brussel we replace <fci-ip> with ``192.168.2.106``

Here we will explain how to add obstacles in the environment with MoveIt, make a plan with MoveIt, and let the robot follow this plan.  


Getting started with MoveIt
----------------------------

First of all, we advice you to follow the tutorials on the |moveit-melodic-tutorials|. 
In these tutorials you work with the Panda robot in rviz. 

.. |moveit-melodic-tutorials| raw:: html

    <a href="http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html" target="_blank">MoveIt website</a>


Test controller-planner 1
-------------------------------

:raw-html:`<font color="blue">  Kelly TO DO: Give a name to the planner. 
Here you want to show the planner based on the Move Group C++ interface whereby the control is done with
a ROS trajectory controller.  </font>`

Test controller-planner 2
-------------------------------

:raw-html:`<font color="blue">  Kelly TO DO: Give a name to the planner. 
Here you want to show the planner based on the Move Group C++ interface whereby the plan is sent to the control node
and an impedance controller is used to control the robot. Torques are sent to the ROS effort controller.  </font>`

To test that libfranka and franka_ros are installed properly, you can run the franka_example_controllers.


The constrained_base_planning_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


This controller adds obstacles to the simulation. For instance, this controller adds the table on which the arm is based, and a virtual wall between the arm and the computer's screen in order to prevent the arm to hit the screen. If you want to use it, execute this line :

.. code-block:: bash

   roslaunch new_controllers constrained_base_planning_controller.launch

You have to way a little at first for the initialisation (till the terminal prints ``Ready to play !``). Then, you will have to click next on the bottom left in order to create the different obstacles and to move the arm.

You can see on the next image the wall and the table (and also an other obstacle in the middle)

.. image:: ./images/constrained.png
    :align: center

So the planner will take into account the obstacles in the environment and will create a path which avoid the obstacles if it is possible.


How to use other planners
-------------------------

.. _Change_planner :

Changing planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The command in c++ to change the planner is :

.. code-block:: bash

    move_group.setPlannerId("PRMkConfigDefault");

Here, the planner is ``PRMkConfigDefault``, which is the PRM planner. A list of all the planner with their characteristics can be found `here <https://planners-benchmarking.readthedocs.io/en/latest/user_guide/2_motion_planners.html>`_

You can find the list of names for setPlannerId in the file located at this place : ``panda_moveit_config/config/ompl_planning.yaml``

.. _Differences :

Differences between the planners
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We wanted to try several planners in order to see the differences between them. So we executed the same path with different planners to see their differences. The main problem is that every planner is random, so for the same path and planner, we obtain different results.

.. _PRM :

Planner PRM
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/PRM1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/PRM2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _RRT_Connect :

Planner RRT Connect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: ./images/RRTconnect1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: ./images/RRTconnect2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _RRT* :

Planner RRT*
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: ./images/RRTstar1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: ./images/RRTstar2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: ./images/RRTstar3.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: ./images/RRTstar4.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: ./images/RRTstar5.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _TRRT :

Planner TRRT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <video width="" height="" controls>
        <source src="../../../source/robot_arm_start/videos/cup.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>

