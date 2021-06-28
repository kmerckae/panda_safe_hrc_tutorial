.. _Different_planners:

=========================
How to use other planners
=========================

.. _Change_planner :

Changing planner
================

The command in c++ to change the planner is :

.. code-block:: bash

    move_group.setPlannerId("PRMkConfigDefault");

Here, the planner is ``PRMkConfigDefault``, which is the PRM planner. A list of all the planner with their characteristics can be found `here <https://planners-benchmarking.readthedocs.io/en/latest/user_guide/2_motion_planners.html>`_

You can find the list of names for setPlannerId in the file located at this place : ``panda_moveit_config/config/ompl_planning.yaml``

.. _Differences :

Differences between the planners
================================

We wanted to try several planners in order to see the differences between them. So we executed the same path with different planners to see their differences. The main problem is that every planner is random, so for the same path and planner, we obtain different results.

.. _PRM :

Planner PRM
===========

.. figure:: images/PRM1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/PRM2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _RRT_Connect :

Planner RRT Connect
===================

.. figure:: images/RRTconnect1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/RRTconnect2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _RRT* :

Planner RRT*
============

.. figure:: images/RRTstar1.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/RRTstar2.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/RRTstar3.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/RRTstar4.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. figure:: images/RRTstar5.png
    :align: center

    Time to find the path : undefined                Time to execute : undefined

.. _TRRT :

Planner TRRT
============

.. raw:: html

    <video width="" height="" controls>
        <source src="../../../source/robot_arm_start/videos/cup.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>
