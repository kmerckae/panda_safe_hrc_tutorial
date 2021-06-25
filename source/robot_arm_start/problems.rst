.. _Problems:

============================================
The different problems we were confronted to
============================================

During the utilisation of the panda arm, we went through different problems that we will list here.

.. _Visualisation:

Problem of visualisation of the trajectory
==========================================

This problem was discover by the fact that in the first example of the `MoveIt! tutorial <http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html>`_, the trajectory of the robot is shown, like in the next image, but not in our projects :

.. image:: images/move_group_interface_tutorial_start_screen.png
    :align: center

In order to solve this problem, we had to update ``panda_moveit_config`` and also add the gripper on the robot. Actually, we used the robot without the gripper and that's why we did not see the trajectory.

.. _Prompt:

Problem with VisualTools.prompt function
========================================

On the :ref:`constrained_base_planning_controller example<Constrained_base_planning_controller>`,the terminal print a text and waits for you to press the ``next`` button. But when we execute the code, the terminal does not print the phrase it should. This problem was just a missing output=screen in the node of path_planner_thomas.
