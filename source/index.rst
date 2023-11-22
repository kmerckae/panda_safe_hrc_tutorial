.. panda robot documentation master file, created by
   sphinx-quickstart on Tue Jun  8 09:57:25 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. role:: raw-html(raw)
    :format: html

Safe Human-Robot Collaboration
===============================

We want to provide a ROS-based open source code framework that enables users to test our proposed planning and control framework for exemplary numerical and experimental validation cases on the 7DOF Franka Emika Panda robot, and in realistic Human-Robot Collaboration (HRC) scenarios with Vicon motion capture data and Stereolabs ZED2 data.

.. note:: The explanation on this website is still under construction. 
            The GitHub repository, https://github.com/panda-brubotics, we are referring to is still private, but will be made public when all relevant articles are published.  
            People from VUB and CU Boulder that are invited to the private repo may not share the content with others.
            For questions, you can always contact Kelly Merckaert: kelly.merckaert@vub.be.

.. .. image:: safeHRC-ERG-vicon.png
..     :align: center
..     :width: 450px

.. |youtube_safeHRC_ERG_vicon| raw:: html

    <a href="https://youtu.be/UzbhMzcKSbE?si=UJyQCnraBKLG9WBJ" target="_blank">   (video) </a>

.. |article_safeHRC_ERG_vicon| raw:: html

    <a href="https://www.sciencedirect.com/science/article/pii/S0736584521001022?via%3Dihub" target="_blank">   (article) </a>


.. figure:: safeHRC-ERG-vicon.png
   :scale: 50%
   :align: center

   We show how a robotic manipulator can reactively avoid collisions with a human by employing the trajectory-based Explicit Reference Governor (ERG) and the Vicon motion capture system. |article_safeHRC_ERG_vicon| |youtube_safeHRC_ERG_vicon| 


.. figure:: safeHRC-RRT-ERG-ZED2.png
   :align: center

   We show how a robotic manipulator can reactively reach its end-effector pose references and avoid collisions with the static cluttered environment and humans by employing a Rapidly-exploring Random Tree (RRT) and trajectory-based ERG algorithm, using the Stereolabs ZED2 camera to detect the obstacles. 

.. This tutorial explains different ways of safe Human-Robot Collaboration (HRC),
.. from safe human-robot coexistence to safe physical human-robot interaction.
.. We explain how to validate the experiments on a Franka Emika Panda robot arm
.. and which sensing systems can be used. For people that don't have the real robot arm, 
.. we explain how to validate the algorithms in the Gazebo simulation environment.  

.. .. |GPL-3-license| raw:: html

..    <a href="https://www.gnu.org/licenses/gpl-3.0.nl.html" target="_blank">GPL-3 license</a>



.. .. admonition:: todo

..   *  include wrap up youtube video
..   *  add |GPL-3-license| to ALL files
..   *  make the GitHub repo public
..   *  change the link to the GitHub repo in all the code block bashes 


-----------------
Table of contents
-----------------

.. toctree::
   :maxdepth: 1

   doc/introduction/introduction
   doc/panda_system/panda_system
   doc/simulation/simulation
   doc/software/software
   doc/hardware/hardware
   doc/archive/archive

-----------------
References
-----------------

[1] K. Merckaert, B. Convens, C. Wu, A. Roncone, M. M. Nicotra, and B. Vanderborght, **Real-time motion control of robotic manipulators for safe human-robot coexistence**, *Robotics and Computer-Integrated Manufacturing*, vol. 73, pp. 102223, Feb. 2022, |10.1016/j.rcim.2021.102223|.

[2] K. Merckaert, B. Convens, M. M. Nicotra, and B. Vanderborght, **Real-Time Constraint-Based Planning and Control of Robotic Manipulators for Safe Human-Robot Collaboration**, *Robotics and Computer-Integrated Manufacturing*, vol. X, pp. X, Month Year, DOI.


.. |10.1016/j.rcim.2021.102223| raw:: html

    <a href="https://doi.org/10.1016/j.rcim.2021.102223" target="_blank">10.1016/j.rcim.2021.102223</a>






.. toctree::
   :maxdepth: 1

   AI/purpose
   AI/data
   AI/train


