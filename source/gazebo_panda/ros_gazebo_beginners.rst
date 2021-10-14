.. _ROS_Gazebo_Beginner:

ROS and Gazebo First-Time Users
======================================

.. role:: raw-html(raw)
    :format: html

Installation requirements
-------------------------

To work with ROS Noetic, you will have to install the operating system Ubuntu 20.04. 

.. |install_ubuntu_20| raw:: html

    <a href="https://ubuntu.com/download/desktop" target="_blank">Ubuntu 20.04</a>

.. |install_ROS_Noetic| raw:: html

    <a href="http://wiki.ros.org/noetic/Installation/Ubuntu" target="_blank">ROS Noetic</a>

.. |InstallingandConfiguringROSEnvironment| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment" target="_blank">environment variables</a>

.. |install_ROS_Control| raw:: html

    <a href="http://wiki.ros.org/ros_control" target="_blank">ROS Control</a>
       
       

*  Install |install_ubuntu_20|

*  Install |install_ROS_Noetic|

   *  Check after installation if the |InstallingandConfiguringROSEnvironment| are set. 

   *  Install |install_ROS_Control|
   
Reading material and tutorials
-------------------------------

ROS and Gazebo 
**************

If you don't know anything about ROS and Gazebo, I recommend you to first read  
:download:`Mastering ROS for Robotics Programming  <doc/JCACACE-MASTERING_ROS_FOR_ROBOTICS_PROGRAMMING_SECOND_EDITION.pdf>`.  
The book is explained for ROS Kinetic, but it is still a very good book if you never worked with ROS before or when you want to refresh your ROS knowledge.  
You can just read the book or maybe better, you can try the tutorials in ROS Noetic. 
Probably you will have to make some minor changes to let it work in ROS Noetic, but that's directly a good practice. 

.. |theconstructsim| raw:: html

    <a href="https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/" target="_blank">The Construct</a>

|theconstructsim| has some very interesting and well-explained ROS courses about the basics of ROS, robot control and navigation, robot manipulation, and so on. 
They have Python and C++ versions of their courses, but please, program everything in C++, since this is the fastest if you want to control robots in real-time. 
For learning algorithms, most of the time Python is used, but for low level control, C++ (and sometimes even C) is preferred.  
Since The Construct uses the Gazebo simulation environment, you will also learn to work with Gazebo by following their tutorials. 
For The Construct tutorials you have to pay. You can start with one month and see if you need more time or not, which depends on the tutorials you want to follow. 

.. |gazebo_sim_tutorials| raw:: html

    <a href="http://gazebosim.org/tutorials" target="_blank">Gazebo tutorials</a>

To learn specifically how to work with Gazebo, it is best to check out the |gazebo_sim_tutorials|. 
Gazebo provides beginner tutorials for first-time Gazebo users, intermediate tutorials to customize your simulation, and advanced tutorials to contribute to Gazebo. 

.. |atlassian_git_version_control| raw:: html

    <a href="https://www.coursera.org/learn/version-control-with-git" target="_blank">Version Control with Git</a>


Git version control
*******************

I really recommend Git version control once you are working on software. 
Follow the Atlassian |atlassian_git_version_control| course and learn to work with it via the command line 
to improve group work and also keep track of the changes you make on your own projects! 
The course doesn't assume any previous knowledge of Git and builds a strong conceptual understanding of the technology,
whereafter you will be able to confidently dig deeper on any version control topic that interests you. 
Atlassian also provides a very helpful |atlassian_git_cheat_sheet|! 

.. |atlassian_git_cheat_sheet| raw:: html

    <a href="https://www.atlassian.com/git/tutorials/atlassian-git-cheatsheet" target="_blank">Git cheat sheet</a>




