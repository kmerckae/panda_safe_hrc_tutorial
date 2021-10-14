.. _Gazebo_Panda_Control:

Panda Control
=========================

.. role:: raw-html(raw)
    :format: html

.. admonition:: todo

	Make updated GitHub repository with ROS Noetic on Ubuntu 20.04 and rewrite this part of the tutorial.

.. |Gazebo_sim_ROS_control| raw:: html

    <a href="http://gazebosim.org/tutorials/?tut=ros_control" target="_blank">Gazebo ROS Control tutorial</a>

To start with the control of the Panda robot in Gazebo, I would also recommend to take a look at the |Gazebo_sim_ROS_control|.

Position control 
-----------------

1)  Adapt the **urdf** folder in your **panda_description** package:

    *  panda_arm.xacro: add the transmission elements

    *  hand.xacro: add the transmission elements

    *  panda.gazebo.xacro: add the gazebo_ros_control plugin

2) Adapt two run_depend in the **package.xml** file of your **panda_gazebo** package:

    *  gazebo_plugins
    
    *  gazebo_ros_control

3) Make a catkin package called **panda_control** in the src folder of you ros_ws. Once in your panda_control package,

    *  make a **config** folder:

        *  add a yaml file called panda_positioncontrol.yaml in which you add a joint_state_controller and joint_position_controllers -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/config
    
    *  make a **src** folder:

        *  add a cpp file called panda_positioncontrol_jointspace.cpp in which you make a subscriber (for the current joint states) 
           and a publisher (to publish the reference joint angles) -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/src
    
    *  make a **launch** folder:
        
        *  add a launch file called panda_positioncontrol.launch in which you load the yaml and launch the controller_spawner and the robot_state_publisher 
           -> check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control/launch
    
    *  your **package.xml** file should contain

        *  <buildtool_depend>catkin</buildtool_depend>

        *  <run_depend>controller_manager</run_depend>

        *  <run_depend>joint_state_controller</run_depend>

        *  <run_depend>robot_state_publisher</run_depend>

        *  <run_depend>effort_controllers</run_depend>

        *  <run_depend>roscpp</run_depend>

    *  your CMakeLists.txt should contain

        *  add_executable(panda_positioncontrol_jointspace src/panda_positioncontrol_jointspace.cpp)

        *  target_link_libraries(panda_positioncontrol_jointspace ${catkin_LIBRARIES} )

        *  check this file in the current version of my repository: constrained_control_robotarm/ros_ws/src/panda_control, 
           in this stage you will *not need*: find_package(...), include in include_directories(...), add_library(...), 
           add_dependencies(...), target_link_libraries(...),  the other add_executable(...) and target_link_libraries(...)

4) In your **panda_gazebo** package

    *  Add another run_depend in the **package.xml** file

        *  panda_control (the catkin package you just made)

5) in the terminal: 
   
   .. code-block:: bash

      roslaunch panda_gazebo panda_positioncontrol_jointspace.launch

   *  you will be asked to enter a reference in joint space for the Panda arm and a finger displacement for the Panda hand

   *  enter these 7 joint angles and the two values for the finger displacement

   *  the robot will go to this reference configuration (maybe with some oscillations)


If this works, you can control the Panda robot by publishing joint angles (see your cpp file).

.. admonition:: todo

	Show video.

Effort control
---------------

Control the robot by publishing torques and give task space reference (add inverse kinematics function) instead of joint space reference.

Joint space reference
*********************

In the previous step you based yourself on panda_positioncontrol in joint space:

*  panda_positioncontrol.yaml where the **effort_controllers/JointPositionController** is used (file in panda_control package)

*  panda_positioncontrol_jointspace.cpp  (file in panda_control package)

*  panda_positioncontrol.launch (file in panda_control package)

*  you launched it with: 

   .. code-block:: bash

      roslaunch panda_gazebo panda_positioncontrol_jointspace.launch (launch file in panda_gazebo package)

You can also try panda_effortcontrol in joint space, check the following files:

*  panda_effortcontrol.yaml where the **effort_controllers/JointEffortController** is used (file in panda_control package)

*  panda_effortcontrol_jointspace.cpp  (file in panda_control package)

*  panda_effortcontrol.launch (file in panda_control package)

*  you launch it with: 

   .. code-block:: bash

     roslaunch panda_gazebo panda_effortcontrol_jointspace (launch file in panda_gazebo package)

.. admonition:: todo

	Show video.

Task space reference
********************

You can also try panda_positioncontrol in task space (therefore you need inverse kinematics function, I used the |orocos_KDL_library| to do this)

*  panda_positioncontrol.yaml where the **effort_controllers/JointPositionController** is used (file in panda_control package)

*  panda_positioncontrol_taskspace.cpp (file in panda_control package)

*  panda_positioncontrol.launch (file in panda_control package)

*  you launch it with: 

   .. code-block:: bash

     roslaunch panda_gazebo panda_positioncontrol_taskspace (launch file in panda_gazebo package)

You can also try panda_effortcontrol in task space, check the following files:

*  panda_effortcontrol.yaml where the **effort_controllers/JointEffortController** is used (file in panda_control package)

*  panda_effortcontrol_taskspace.cpp (file in panda_control package)

*  panda_effortcontrol.launch (file in panda_control package)

*  you launch it with: 

   .. code-block:: bash

     roslaunch panda_gazebo panda_effortcontrol_taskspace (launch file in panda_gazebo package)

.. admonition:: todo

	Show video.

.. |orocos_KDL_library| raw:: html

    <a href="https://www.orocos.org/kdl.html" target="_blank">KDL library </a>

For the programs in task space, I used the |orocos_KDL_library| (since they have an inverse kinematics function). Check my stabilizing_control library for it (in panda_control package in the include folder). 
