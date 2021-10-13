.. _ROS_Gazebo_Beginner:

ROS and Gazebo First-Time Users
======================================

.. role:: raw-html(raw)
    :format: html

.. admonition:: todo

	Rewrite clearly for Ubuntu 20.04 and ROS Noetic.

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


I really recommend Git version control once you are working on software. 
Follow the Atlassian |atlassian_git_version_control| course and learn to work with it via the command line 
to improve group work and also keep track of the changes you make on your own projects! 
The course doesn't assume any previous knowledge of Git and builds a strong conceptual understanding of the technology,
whereafter you will be able to confidently dig deeper on any version control topic that interests you. 











Step 1: Visualize the Panda robot in Gazebo


    If you want to work with git, make a repository  I also highly recommend to work with git!!!! 
    Make a catkin workspace in your repository (or in a separate folder as you want). If you have made a repository, then you will have to do the following in the terminal. 

        cd repository_name/
        mkdir -p ros_ws/src
        cd ros_ws
        source /opt/ros/melodic/setup.sh
        catkin_init_workspace src

    The first thing you want to do with the robot is to visualize the Panda robot in Rviz (Robot Visualization). To do this, we need the kinematics and the meshes of the robot. These two can be found on Franka Emika's github account. 
        Make a catkin package called panda_description in the src folder of your ros_ws.  

            cd repository_name/ros_ws/src/
            catkin_create_pkg panda_description

        In your panda_description package: 
            You can compare with my panda_description package I had at the moment I visualized the robot in Rviz. Therefore, go to my repository (constrained_control_robotarm), click on "98 commits", go to commits on 16 July 2019, click on "view Panda in Rviz", click on "Browse files" . Now you can see the folders, packages, files I had at that moment. 
            Make a folder called urdf. Copy the urdf and xacro files that are inside the robots folder in franka_ros to your urdf folder. I also did this, but there was no dual_panda example at that moment. 
            Make a folder called meshes. Copy the folders collision and visual from my code into your meshes folder. 
            Make a folder called launch. Copy the two files that are in my launch folder into your launch folder. 
            Check the package.xml and the CMakeLists.txt files. Compare them with the ones I had at that moment and add the lines (e.g. dependencies) you don't have in your files. 
        Launch the launch file by entering in the terminal roslaunch panda_description panda_rviz.launch.  Rviz will pop up and will show a Panda robot that is standing still. You can move the joints of this Panda robot by the joint_state_publisher GUI.
    The second thing you want to do is to visualize the Panda robot in Gazebo. This is an interesting Gazebo tutorial to follow.  Therefore we need to include the robot dynamic parameters to the URDF file. Since Franka doesn't provide the dynamic parameters, I inserted the dynamic parameters from Erdal Pekel. 
          You can compare with my files I had at the moment I visualized the robot in Gazebo. Therefore, go to my repository (constrained_control_robotarm), click on "98 commits", go to commits on 17 July 2019, click on "view Panda in Gazebo", click on "Browse files" . Now you can see the folders, packages, files I had at that moment. 
        In your panda_description package: 
            Adapt the urdf folder in your panda_description package. (take a look at my files from that moment!)
                panda_arm_hand.urdf: rigidly fix the base to the Gazebo world
                hand.xacro: add inertial values  
                panda_arm.xacro: add inertial values + add joint damping 
                panda.gazebo.xacro: new file with gazebo specifications
                panda_arm_hand.urdf.xacro: include panda.gazebo.xacro  
        Make a catkin package called panda_gazebo in the src folder of your ros_ws. 
        In your panda_gazebo package:
            Make a folder called worlds. Create a world file including a ground plane, a light source (sun), and a camera at a certain position and orientation
            Make a folder called launch. Create a launch file. 
            Check the package.xml and the CMakeLists.txt files. Compare them with the ones I had at that moment and add the lines you don't have. 
        Launch the launch file by entering the terminal roslaunch panda_gazebo panda_world.launch.  Gazebo will pop up showing a Panda robot. Despite there being no intentional disturbances in the physics simulator by default, numerical errors should start to build up and cause the Panda robot to move a bit in an uncontrolled way. (In this stage there is no control added to the simulation yet.)

There is a recently accepted paper (see attachment) in which the authors identified the dynamic parameters of the Panda robot. I implemented them in my code 4 months ago, check the last version of  panda_arm.xacro  in panda_description. .

Step 2: control the Panda robot in Gazebo
For this step, I would also recommend to take a look at the Gazebo ROS Control tutorial.

1) In your panda_description package:

    Adapt he urdf folder in your panda_description package.
        panda_arm.xacro: add the transmission elements
        hand.xacro: add the transmission elements
        panda.gazebo.xacro: add the gazebo_ros_control plugin

2) In your panda_gazebo package:

    Add two run_depend in the package.xml file
        gazebo_plugins
        gazebo_ros_control

3) Make a catkin package called panda_control in the src folder of you ros_ws. Once in your panda_control package,

    make a config folder:
        add a yaml file called panda_positioncontrol.yaml in which you add a joint_state_controller and joint_position_controllers -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/config
    make a src folder:
        add a cpp file called panda_positioncontrol_jointspace.cpp in which you make a subscriber (for the current joint states) and a publisher (to publish the reference joint angles) -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/src
    make a launch folder:
        add a launch file called panda_positioncontrol.launch in which you load the yaml and launch the controller_spawner and the robot_state_publisher -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/launch
    your package.xml file should contain
        <buildtool_depend>catkin</buildtool_depend>
        <run_depend>controller_manager</run_depend>
        <run_depend>joint_state_controller</run_depend>
        <run_depend>robot_state_publisher</run_depend>
        <run_depend>effort_controllers</run_depend>
        <run_depend>roscpp</run_depend>
    your CMakeLists.txt should contain
        add_executable(panda_positioncontrol_jointspace src/panda_positioncontrol_jointspace.cpp)
        target_link_libraries(panda_positioncontrol_jointspace ${catkin_LIBRARIES} )
        check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control, in this stage you will not need: find_package(...), include in include_directories(...), add_library(...), add_dependencies(...), target_link_libraries(...),  the other add_executable(...) and target_link_libraries(...)

4) In your panda_gazebo package

    Add another run_depend in the package.xml file
        panda_control (the catkin package you just made)

5) in the terminal: roslaunch panda_gazebo panda_positioncontrol_jointspace.launch

    you will be asked to enter a reference in joint space for the Panda arm and a finger displacement for the Panda hand
    enter these 7 joint angles and the two values for the finger displacement
    the robot will go to this reference configuration (maybe with some oscillations)


If this works, you can control the Panda robot by publishing joint angles (see your cpp file).

Step 3: control the robot by publishing torques and give task space reference (add inverse kinematics function) instead of joint space reference.

In the previous step you based yourself on panda_positioncontrol in joint space:

    panda_positioncontrol.yaml where the effort_controllers/JointPositionController is used (file in panda_control package)
    panda_positioncontrol_jointspace.cpp  (file in panda_control package)
    panda_positioncontrol.launch (file in panda_control package)
    you launched it with: roslaunch panda_gazebo panda_positioncontrol_jointspace.launch (launch file in panda_gazebo package)

You can also try panda_effortcontrol in joint space, check the following files:

    panda_effortcontrol.yaml where the effort_controllers/JointEffortController is used (file in panda_control package)
    panda_effortcontrol_jointspace.cpp  (file in panda_control package)
    panda_effortcontrol.launch (file in panda_control package)
    you launch it with: roslaunch panda_gazebo panda_effortcontrol_jointspace (launch file in panda_gazebo package)

You can also try panda_positioncontrol in task space (therefore you need inverse kinematics function, I used the KDL library to do this)

    panda_positioncontrol.yaml where the effort_controllers/JointPositionController is used (file in panda_control package)
    panda_positioncontrol_taskspace.cpp (file in panda_control package)
    panda_positioncontrol.launch (file in panda_control package)
    you launch it with: roslaunch panda_gazebo panda_positioncontrol_taskspace (launch file in panda_gazebo package)

You can also try panda_effortcontrol in task space, check the following files:

    panda_effortcontrol.yaml where the effort_controllers/JointEffortController is used (file in panda_control package)
    panda_effortcontrol_taskspace.cpp (file in panda_control package)
    panda_effortcontrol.launch (file in panda_control package)
    you launch it with: roslaunch panda_gazebo panda_effortcontrol_taskspace (launch file in panda_gazebo package)


For the programs in task space, I used the KDL library (since they have an inverse kinematics function). Check my stabilizing_control library for it (in panda_control package in the include folder). 