=================
Introduction
=================

.. role:: raw-html(raw)
    :format: html

.. |CTU_MRS| raw:: html

    <a href="https://ctu-mrs.github.io/" target="_blank">CTU MRS open source platform</a>

To exploit multicore processor capabilities, we have developed a ROS-based system architecture whereby we have split all separate tasks (e.g., pre-stabilizing control, trajectory-based ERG, reference selector, RRT-Connect planner, visualizations) over different ROS nodes such that they can all run in parallel.

Note that exploiting these multicore processor capabilities is extremely important for control algorithms that run at high rates and require data from multiple other running processes while the robot needs to receive control commands at 1kHz and stops when there are too many data losses. We did not do this from the start, but the more high-computational demanding processes we added to our framework (e.g., planning, collision checking), the more communication delays we encountered whereby the robot stopped. The only solution to this problem was to run all the different tasks in parallel, which also made our proposed architecture modular. 

We have grouped these ROS nodes in different ROS packages. All these ROS packages can be found in https://github.com/panda-brubotics which will be made public in the (near) future. 

To develop this ROS-based system architecture, we have taken inspiration from the |CTU_MRS|, i.e., the Multi-robot Systems Group robotics lab at the Czech Technical University in Prague who mostly works with multi-rotor drones, and for them specifically, developed a control, estimation, and simulation platform enabling real-world and replicable simulations and experiments.

---------------------------
Mandatory software to learn
---------------------------

Ubuntu
--------
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

To start with ROS, you have to install |install_ROS_Noetic| and follow the tutorials on the |ROS_tutorials|. 

We also recommend to read  
:download:`Mastering ROS for Robotics Programming  <JCACACE-MASTERING_ROS_FOR_ROBOTICS_PROGRAMMING_SECOND_EDITION.pdf>`.  
The book is explained for ROS Kinetic, but it is still a very good book if you never worked with ROS before or when you want to refresh your ROS knowledge.  
You can read the book and try the tutorials in ROS Noetic. You will have to make some minor changes to let it work in ROS Noetic, but that's directly a good practice. 

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

|tmux| is a command-line utility that allows splitting a terminal to multiple panels and creating windows (tabs). It runs entirely in the command line. It is scriptable, which makes it ideal for automating processes, where multiple programs are launched simultaneously.

Tmuxinator - automating tmux
----------------------------

.. |tmuxinator| raw:: html

    <a href="https://github.com/tmuxinator/tmuxinator" target="_blank">tmuxinator</a>

Tmux itself is very powerful, |tmuxinator| uses .xml files containing a description of a tmux session. It allows us to define and automate complex multi-terminal setups for, e.g., development (one session per program) and simulations. All our validation startup scripts are written for tmuxinator.

C++
------

.. |Cpp_beginners| raw:: html

    <a href="https://www.youtube.com/watch?v=vLnPwxZdW4Y" target="_blank">C++ for beginners</a>

.. |Cpp_indepth| raw:: html

    <a href="https://www.youtube.com/user/lefticus1" target="_blank">C++ weekly by Jason Turner series</a>



All low-level control software that is intended to run in real-time is preferred to be written in C++. Although ROS/ROS2 natively supports also Python, well-written C++ provides significantly better performance. Therefore we recommend to learn C++ and get used to programming with it.

We advise to follow the |Cpp_beginners| tutorial to start with and to take a look at the |Cpp_indepth| to go much deeper into C++. We also recommend to read  
:download:`Effective Modern C++  <Effective-Modern-C++.pdf>` to improve your use of C++11 and C++14.  


.. Gazebo
.. -----------------------------
.. .. |gazebo_sim_tutorials| raw:: html

..     <a href="https://gazebosim.org/docs" target="_blank">Gazebo tutorials</a>

.. Gazebo is To learn specifically how to work with Gazebo, it is best to check out the |gazebo_sim_tutorials|. 
.. Gazebo provides beginner tutorials for first-time Gazebo users, intermediate tutorials to customize your simulation, and advanced tutorials to contribute to Gazebo. 




-------------------------
How to install
-------------------------
.. |safe_panda_system_package| raw:: html

    <a href="https://github.com/panda-brubotics/safe_panda_system" target="_blank">safe_panda_system package</a>
    
    

The starting point is the |safe_panda_system_package| which has an
automated install-script that installs ROS, the git and tmux dependencies, the
panda_core package, and creates a ROS workspace called panda_workspace.

.. code-block:: bash

    bash
    cd /tmp
    echo '
    GIT_PATH=~/git
    mkdir -p $GIT_PATH
    cd $GIT_PATH
    sudo apt-get -y install git
    git clone https://github.com/panda-brubotics/safe_panda_system.git
    cd safe_panda_system
    git checkout master
    git pull
    ./install.sh -g $GIT_PATH
    source ~/.bashrc' > clone.sh && source clone.sh
   



Since our framework can be seen as a specific extension of other existing
frameworks and libraries, the panda_core package installs all required third-party packages such as libfranka and franka_ros to employ the code on the
Franka Emika Panda robot, moveit for planning and collision checking purposes,
rviz for visualizations, the zed interfaces to use the Stereolabs ZED2 camera,
and the vicon bridge to use the Vicon motion capture system. Besides these
third-party packages, it also installs the packages necessary for the proposed
planning and control framework.


-------------------------
How to test 
-------------------------

.. note:: Kelly tested the code on a desktop with Ubuntu 18.04 and franka_ros version 0.7.0. However, it also works with Ubuntu 20.04 and the newer franka_ros version. The required adaptations to run it on the newest software will be documented in January 2025. 

libfranka examples
-------------------
Navigate to the libfranka examples and take a look at the available examples:

.. code-block:: bash

    cd panda_workspace/src/panda_core/libfranka/build/examples
    ls

For the examples where the robot will move, you should put the robot in the activated state (i.e., when the lights are blue). Try for example: 

.. code-block:: bash

    ./generate_elbow_motion.cpp <robot_ip>

franka_ros examples
--------------------
Navigate to the franka_ros examples and take a look at the available examples:

.. code-block:: bash

    cd panda_workspace/src/panda_core/ros_packages/franka_ros/franka_example_controllers/launch
    ls

For the examples where you only get data from the robot, the robot can be in the interactive state (i.e., when the lights are white). Try for example: 

.. code-block:: bash

    roslaunch franka_example_controllers model_example_controller.launch robot_ip:=<robot_ip>

For almost all examples where you will let the robot move, the robot should initially be in its basic configuration, therefore run the following line in the activated state (i.e., when the lights are blue):

.. code-block:: bash

    roslaunch franka_example_controllers move_to_start.launch robot_ip:=<robot_ip>

From there the other examples can be executed in the activated state: 

.. code-block:: bash

    roslaunch franka_example_controllers elbow_example_controller.launch robot_ip:=<robot_ip>

Kelly examples
--------------

For these examples you only need the Panda robot (no cameras are required). 

Navigate to Kelly's examples in the panda_testing package.

.. code-block:: bash

    cd panda_workspace/src/panda_core/ros_packages/panda_testing/tmux_scripts/kelly
    ls


All examples work with tmux sessions.

*   To kill a separate window of a tmux session, use ``CTRL+C`` in that specific window
*   To kill all windows of a tmux session, use ``CTRL+A+K``. (possible that this shortcut doesn't work yet)
*   Another way to kill all tmux windows is to use ``CTRL+C`` in one window and then write 

    .. code-block:: bash

        tmux kill-server 

.. note:: Note to Kelly: add videos and figures to the examples. 

0) Basic examples
^^^^^^^^^^^^^^^^^^^^^
In 46_prestabilizing_control a pure PD+g controller is employed. To test it, put the robot in the activated mode, navigate to the test folder, and run the start shell. 

.. code-block:: bash

    cd 46_prestabilizing_control
    ./start-test.sh

You will see that the robot moves with small steps defined in the small_steps.cpp in the panda_tasks repository.

If you don't see this and you get connection errors, then this is probably due to the fact that the robot-ip is hard-coded in many files. Search for Get Franka robot model and robot state and update the robot ip.


.. note:: Debugging is made easier with this tmux structure, since you can see directly in a single window what is able to run and what is not. You have to navigate in the tmux session through all the windows via ``CTRL+A+N`` and check what errors/warnings you receive. Instead of launching the separate launch files to debug, it is better to comment the windows that don't work in the session-test.yaml before starting the tmux session such that you can check where it goes wrong. 

.. note:: **unable to connect to service: [Errno 111]**
    
    If you get this error, this is probably because of a wrong robot (which is still hard-coded in many programs and should become adaptable in the future). 
    
    Check the following: 

    *   In the session-test.yaml file of this *46_prestabilizing_control* test you see in the third window the odometry_manager.cpp is asked to run.
    
    *   When you start this test, navigate in the tmux session to this third window. Probably you get a connection error here. If this is true, then go to the odometry_manager.cpp file. You will see under *Get Franka robot model and robot state* that my robot_ip is hard-coded. Adapt this to your robot_ip and start the tmux file again. 
    *   When you start this test again and navigate to this third (odometry) window, there shouldn't be an error anymore. What do you see when you navigate to the fourth (task) window? Probably a similar connection problem. Go to panda_tasks repository and to the small_steps.cpp, adapt the robot_ip under *Get Franka robot model and robot state*. 
    *   Run again the tmux session and test similar things as before, now with window 4 (task) and window 5 (planner). 


.. |franka_research3_compatibility| raw:: html

    <a href="https://frankaemika.github.io/docs/compatibility.html" target="_blank">compatibility requirements</a>

.. note:: **Issues because of using the newest Franka Research 3 version**
    
    It is possible that you get connection issues because of using the Franka Research 3 version for which the franka_ros and moveit versions should be updated to their latest, given the |franka_research3_compatibility|. 
    
    This issue can come up when the launch requires something like a controller manager as opposed to the odometry manager.

    With the version changes, the next issue that can come up is a timing issue: *Failed to fetch current robot state.* The hunch is that this has to do with some extra checks on timing in the new moveit version. 

1) Test collision checks by moving Panda robot yourself
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To test MoveIt's collision checking functionality by moving the robot yourself in the interactive state, navigate to the test case folder and run the shell.

.. code-block:: bash

    cd 30_collisioncheck_caseA
    ./start-test.sh

You will see a tmux session and an RViz screen opening. Once everything is loaded, you can start moving the robot. The shortest distance between the robot and the obstacle scene should be visualized. Once the robot is in collision with the obstacle scene, this should also be visualized in RViz and printed in the terminal of the planningscene_collisioncheck window. (Navigate via ``CTRL+A+N`` to this window.)

*   To run the test in obstacle scene B, you have to navigate to 31_collisioncheck_caseB.
*   To run the test in obstacle scene C, the Octomap, you have to navigate to 32_collisioncheck_octomap.


2) Test generating octopoints by moving Panda robot yourself
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To test the octopoints generation algorithm by moving the robot yourself, put the Panda robot in the interactive mode, navigate to the test case folder, and run the test shell. 

.. code-block:: bash

    cd 33_generate_octopoints
    ./start-test.sh

You will see a tmux session and an RViz screen opening. Once everything is loaded, you can start moving the robot. You should see pink points, the octopoints, that are generated when moving the robot closer to the octomap, a purple arrow denoting the shortest distance between the robot and the octopoints.



3) Test MoveIt's RRT Connect planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To test MoveIt's RRT Connect planner in obstacle scene A (table, wall, and two cylinders), put the Panda robot in the activated mode, navigate to the test folder, and run the test shell. 

.. code-block:: bash

    cd 20_planner_caseA
    ./start-test.sh

You will see a tmux session and an RViz screen opening. Once everything is loaded you can start the test by starting the timer. Therefore you have to navigate (via ``CTRL+A+N``) in the tmux session to the time_handler window, which should be the third screen. Then start the timer. The command

.. code-block:: bash
    
    rosrun panda_managers time_manager

is saved in the history, so you don't have to type the above yourself in the terminal, but you only have to click the pointing up arrow key on your keyboard and click enter. You should see the real robot moving towards the reference poses. In RViz you should see the same, including the obstacle scene, the references, and the planned paths.

*   To run the test in obstacle scene B, you have to navigate to 21_planner_caseB.
*   To run the test in obstacle scene C, the Octomap, you have to navigate to 26_planner_octomap.

4) Test the trajectory-based ERG
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To test the trajectory-based ERG in obstacle scene A (table, wall, and two cylinders), put the Panda robot in the activated mode, navigate to the test folder, and run the test shell. 

.. code-block:: bash

    cd 22_erg_caseA
    ./start-test.sh

You will see a tmux session and an RViz screen opening. Once everything is loaded (this takes longer than in pure planner case) you can start the test by starting the timer. Check first that all commands are started by navigating through all the windows (via ``CTRL+A+N``) in the tmux session. Once this is okay, you can navigate in the tmux session to the time_handler window and start the timer by clicking the pointing up arrow key on your keyboard (you should see the rosrun panda_managers time_manager command) and clicking enter. You should see the real robot moving towards the reference poses. In RViz you should see the same, including the obstacle scene, the references, the planned paths, and the paths made by connecting the applied references coming from the ERG.

*   To run the test in obstacle scene B, you have to navigate to 23_erg_caseB.
*   To run the test in obstacle scene C, the Octomap, you have to navigate to 27_erg_octomap.



5) Test the RRT Connect + trajectory-based ERG
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To test the combination of the RRT Connect with the trajectory-based ERG in obstacle scene A (table, wall, and two cylinders), put the Panda robot in the activated mode, navigate to the test folder, and run the test shell. 

.. code-block:: bash

    cd 24_selector_caseA
    ./start-test.sh

You will see a tmux session and an RViz screen opening. Once everything is loaded (this takes longer than in pure planner case) you can start the test by starting the timer. Check first that all commands are started by navigating through all the windows (via ``CTRL+A+N``) in the tmux session. Once this is okay, you can navigate in the tmux session to the time_handler window and start the timer by clicking the pointing up arrow key on your keyboard (you should see the rosrun panda_managers time_manager command) and clicking enter. You should see the real robot moving towards the reference poses. In RViz you should see the same, including the obstacle scene, the references, the planned paths (green), and the paths made by connecting the selected references from the reference selector (orange) and the applied references coming from the ERG (blue).

*   To run the test in obstacle scene B, you have to navigate to 25_selector_caseB.
*   To run the test in obstacle scene C, the Octomap, you have to navigate to 28_selector_octomap.
