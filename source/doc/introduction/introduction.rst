=================
Introduction
=================

.. role:: raw-html(raw)
    :format: html

.. |CTU_MRS| raw:: html

    <a href="https://ctu-mrs.github.io/" target="_blank">CTU MRS open source platform</a>

The developed ROS-based system architecture has taken inspiration from the |CTU_MRS|, i.e., the Multi-robot Systems Group robotics lab at the Czech Technical University in Prague who mostly works with multi-rotor drones, and for them specifically, developed a control, estimation, and simulation platform enabling real-world and replicable simulations and experiments.

To exploit multicore processor capabilities, we have split all separate tasks (e.g., pre-stabilizing control, trajectory-based ERG, reference selector, RRT Connect planner, visualizations) over different ROS nodes such that they can all run in parallel. Note that exploiting these multicore processor capabilities is extremely important for control algorithms that run at high rates and require data from multiple other running processes while the robot needs to receive control commands at 1kHz and stops when there are too many data losses. We did not do this from the start, but the more high-computational demanding processes we added to our framework (e.g., planning, collision checking), the more communication delays we encountered whereby the robot stopped. The only solution to this problem was to run all the different tasks in parallel, which also made our proposed architecture modular.

We have grouped these ROS nodes in different ROS packages. All these ROS packages can be found in https://github.com/panda-brubotics which will be made public in the (near) future. 

-------------------------------
Suggested reading for newcomers
-------------------------------

Mandatory software to learn
===========================
Ubuntu
-------
.. |install_ubuntu_20| raw:: html

    <a href="https://ubuntu.com/download/desktop" target="_blank">Ubuntu 20.04</a>

Our software is compatible with |install_ubuntu_20|. We strongly suggest installing a native system, not a virtual one. 

Bash
-------
Bash is the standard shell in Ubuntu. Most of the time, when you work in the terminal, you use bash to run programs, for scripting and to manage your file system.

ROS - Robot Operating System
-----------------------------
.. |ROS_Noetic_lastofficialROS1| raw:: html

    <a href="https://www.openrobotics.org/blog/2020/5/23/noetic-ninjemys-the-last-official-ros-1-release" target="_blank">final release of ROS1</a>

.. |install_ROS_Noetic| raw:: html

    <a href="http://wiki.ros.org/noetic/Installation/Ubuntu" target="_blank">ROS Noetic</a>

.. |InstallingandConfiguringROSEnvironment| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment" target="_blank">environment variables</a>

.. |install_ROS_Control| raw:: html

    <a href="http://wiki.ros.org/ros_control" target="_blank">ROS Control</a>

.. |ROS_tutorials| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials" target="_blank">ROS webpage</a>
       

ROS is a middleware between Ubuntu and C++/Python. Thanks to it, our programs can talk to each other asynchronously. It also allows simple control of your software from the terminal. A lot of utilities for robotics have already been programmed with ROS. Since we do not want to reinvent the wheel and create a planning and
control framework that makes use of existing and well-maintained libraries and
packages, it is clear our framework should be ROS-based. 

Our software is compatible with the |ROS_Noetic_lastofficialROS1|. We will update it in the future such that it will also be compatible with ROS2. 

Getting into ROS is simple. Install |install_ROS_Noetic| and follow the tutorials on the |ROS_tutorials|. 

We also recommend to read  
:download:`Mastering ROS for Robotics Programming  <JCACACE-MASTERING_ROS_FOR_ROBOTICS_PROGRAMMING_SECOND_EDITION.pdf>`.  
The book is explained for ROS Kinetic, but it is still a very good book if you never worked with ROS before or when you want to refresh your ROS knowledge.  
You can just read the book or maybe better, you can try the tutorials in ROS Noetic. Probably you will have to make some minor changes to let it work in ROS Noetic, but that's directly a good practice. 

.. |theconstructsim| raw:: html

    <a href="https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/" target="_blank">The Construct</a>

|theconstructsim| has some very interesting and well-explained ROS courses about the basics of ROS, robot control and navigation, robot manipulation, and so on. 
They have Python and C++ versions of their courses.  

.. Catkin
.. -------
.. Catkin is the package and workspace manager used by ROS. You’ll use it to build the ROS packages and manage their dependencies. Check out our Catkin fundamentals tutorial.

GIT - code versioning system
------------------------------
.. |atlassian_git_version_control| raw:: html

    <a href="https://www.coursera.org/learn/version-control-with-git" target="_blank">Version Control with Git</a>

.. |atlassian_git_cheat_sheet| raw:: html

    <a href="https://www.atlassian.com/git/tutorials/atlassian-git-cheatsheet" target="_blank">Git cheat sheet</a>

GIT is a file versioning system. All our code is stored and versioned using Git and hosted on GitHub. It allows collaborative work between many people and can be managed entirely from the terminal.

Follow the Atlassian |atlassian_git_version_control| course and learn to work with it via the command line. The course doesn't assume any previous knowledge of Git and builds a strong conceptual understanding of the technology, whereafter you will be able to confidently dig deeper on any version control topic that interests you. 

Atlassian also provides a very helpful |atlassian_git_cheat_sheet|! 

TMUX - terminal multiplexer
----------------------------

.. |tmux| raw:: html

    <a href="https://github.com/tmux/tmux" target="_blank">Tmux</a>

|tmux| is a command-line utility that allows splitting a terminal to multiple panels and creating windows (tabs). It runs entirely in the command line. It is scriptable, which makes it ideal for automating processes, where multiple programs are launches simultaneously.

Tmuxinator - automating tmux
----------------------------

.. |tmuxinator| raw:: html

    <a href="https://github.com/tmuxinator/tmuxinator" target="_blank">tmuxinator</a>

Tmux itself is very powerful, |tmuxinator| uses .xml files containing a description of a tmux session. It allows us to define and automate complex multi-terminal setups for, e.g., development (one session per program) and simulations. All our simulation startup script are written for tmuxinator.

C++
------

.. |Cpp_beginners| raw:: html

    <a href="https://www.youtube.com/watch?v=vLnPwxZdW4Y" target="_blank">C++ for beginners</a>

.. |Cpp_indepth| raw:: html

    <a href="https://www.youtube.com/user/lefticus1" target="_blank">C++ weekly by Jason Turner series</a>



All low-level control software that is intended to run in real-time is preferred to be written in C++. Although ROS/ROS2 natively supports also Python, well-written C++ provides significantly better performance. Therefore we recommend to learn C++ and get used to programming with it.

We advise to follow the |Cpp_beginners| tutorial to start with and to take a look at the |Cpp_indepth| to go much deeper into C++. 

.. note:: Add book best practices MoveIt. 

.. Gazebo
.. -----------------------------
.. .. |gazebo_sim_tutorials| raw:: html

..     <a href="https://gazebosim.org/docs" target="_blank">Gazebo tutorials</a>

.. Gazebo is To learn specifically how to work with Gazebo, it is best to check out the |gazebo_sim_tutorials|. 
.. Gazebo provides beginner tutorials for first-time Gazebo users, intermediate tutorials to customize your simulation, and advanced tutorials to contribute to Gazebo. 







How to install
===========================

