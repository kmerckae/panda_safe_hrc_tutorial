Gazebo Simulation
=================

.. role:: raw-html(raw)
    :format: html


.. admonition:: todo

	Make updated GitHub repository with ROS Noetic on Ubuntu 20.04 and rewrite this part of the tutorial.


Create a catkin workspace
-------------------------

.. |atlassian_setting_up_a_repository| raw:: html

    <a href="https://www.atlassian.com/git/tutorials/setting-up-a-repository" target="_blank">set up a repository</a>

If you want to work with Git version control, then you first need to |atlassian_setting_up_a_repository|. 

.. |ROS_create_a_workspace| raw:: html

    <a href="http://wiki.ros.org/catkin/Tutorials/create_a_workspace" target="_blank">catkin workspace</a>

Make a |ROS_create_a_workspace| in your repository. Some people call their catkin workspace catkin_ws others call it ros_ws. 
Below you see the example for ros_ws. Go to the terminal and execute the following lines. 

.. code-block:: bash

   cd path/to/your_repository
   mkdir -p ros_ws/src
   cd ros_ws
   source /opt/ros/melodic/setup.sh
   catkin_init_workspace src


.. _Gazebo_Panda_Visualization:

Visualization
----------------------

Visualize the Panda robot in RViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. |GitHub_Franka_Emika| raw:: html

    <a href="https://github.com/frankaemika/franka_ros/tree/kinetic-devel/franka_description" target="_blank">Franka Emika's GitHub account</a>
    
The first thing you want to do with the robot is to visualize the Panda robot in RViz (i.e. Robot Visualization). 
To do this, we need the kinematics and the meshes of the robot. These two can be found on the |GitHub_Franka_Emika|. 

.. |ROS_create_package| raw:: html

    <a href="http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage" target="_blank">catkin package</a>

Create a |ROS_create_package| called panda_description in the src folder of your ros_ws.  

.. code-block:: bash

    cd path/to/your_repository/ros_ws/src/
    catkin_create_pkg panda_description

In your **panda_description** package: 

*  You can compare with my panda_description package I had at the moment I visualized the robot in Rviz. Therefore, go to my repository (constrained_control_robotarm), 
   click on "98 commits", go to commits on 16 July 2019, click on "view Panda in Rviz", click on "Browse files". 
   Now you can see the folders, packages, files I had at that moment. 
*  Make a folder called **urdf**. Copy the urdf and xacro files that are inside the robots folder in franka_ros to your urdf folder. 
   I also did this, but there was no dual_panda example at that moment. 
*  Make a folder called **meshes**. Copy the folders **collision** and **visual** from my code into your meshes folder. 
*  Make a folder called **launch**. Copy the two files that are in my launch folder into your launch folder. 
*  Check the package.xml and the CMakeLists.txt files. Compare them with the ones I had at that moment and add the lines (e.g. dependencies) you don't have in your files. 


Launch the launch file by entering in the terminal 

.. code-block:: bash

    roslaunch panda_description panda_rviz.launch

Rviz will pop up and will show a Panda robot that is standing still. 
You can move the joints of this Panda robot by the joint_state_publisher GUI.

.. admonition:: todo

	Show video.


Visualize the Panda robot in Gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. |Gazebo_sim_UsingaURDFinGazebo| raw:: html

    <a href="http://gazebosim.org/tutorials/?tut=ros_urdf#Tutorial:UsingaURDFinGazebo" target="_blank">Gazebo tutorial</a>

.. |Erdal_Pekel_Panda_in_Gazebo| raw:: html

    <a href="https://erdalpekel.de/?p=55" target="_blank">Erdal Pekel</a>

Once you can visualize the Panda robot in RViz, it is time to visualize the robot in Gazebo. 
Take also a look to this |Gazebo_sim_UsingaURDFinGazebo| that explains how to use a URDF in Gazebo. 

To visualize the Panda robot in Gazebo, we need to include the robot dynamic parameters to the URDF file. 
Since Franka doesn't provide the dynamic parameters, I inserted the dynamic parameters from |Erdal_Pekel_Panda_in_Gazebo|. 

You can compare with my files I had at the moment I visualized the robot in Gazebo. 
Therefore, go to my repository (constrained_control_robotarm), click on "98 commits", go to commits on 17 July 2019, click on "view Panda in Gazebo", click on "Browse files" . 
Now you can see the folders, packages, files I had at that moment. 

*  Adapt the **urdf** folder in your **panda_description** package. (take a look at my files from that moment!)

    *  panda_arm_hand.urdf: rigidly fix the base to the Gazebo world
    *  hand.xacro: add inertial values  
    *  panda_arm.xacro: add inertial values + add joint damping 
    *  panda.gazebo.xacro: new file with gazebo specifications
    *  panda_arm_hand.urdf.xacro: include panda.gazebo.xacro  

*  Make a catkin package called panda_gazebo in the src folder of your ros_ws. 
*  In your **panda_gazebo** package:

    *  Make a folder called **worlds**. Create a world file including a ground plane, a light source (sun), and a camera at a certain position and orientation
    *  Make a folder called **launch**. Create a launch file. 
    *  Check the package.xml and the CMakeLists.txt files. Compare them with the ones I had at that moment and add the lines you don't have. 

*  Launch the launch file by entering the terminal 

   .. code-block:: bash

     roslaunch panda_gazebo panda_world.launch
    
   Gazebo will pop up showing a Panda robot. Despite there being no intentional disturbances in the physics simulator by default, numerical errors should start to build up and cause the Panda robot to move a bit in an uncontrolled way. (In this stage there is no control added to the simulation yet.)

.. admonition:: todo

	Show video.

.. note:: In :download:`Dynamic Identification of the Franka Emika Panda Robot with Retrieval of Feasible Parameters Using Penalty-Based Optimization <PandaDynamicIdentification.pdf>`
          and :download:`its supplementary material <PandaDynamicIdentification_SupplementaryMaterial.pdf>`, 
          the authors identified the dynamic parameters of the Panda robot. 
          I implemented them in my code some months ago, check the last version of panda_arm.xacro in panda_description.


.. _Gazebo_Panda_Control:

ROS Control
-----------------

.. |Gazebo_sim_ROS_control| raw:: html

    <a href="http://gazebosim.org/tutorials/?tut=ros_control" target="_blank">Gazebo ROS Control tutorial</a>

To start with the control of the Panda robot in Gazebo, I would also recommend to take a look at the |Gazebo_sim_ROS_control|.

Position control 
^^^^^^^^^^^^^^^^^^

*  Adapt the **urdf** folder in your **panda_description** package:

    *  panda_arm.xacro: add the transmission elements
    *  hand.xacro: add the transmission elements
    *  panda.gazebo.xacro: add the gazebo_ros_control plugin

*  Adapt two run_depend in the **package.xml** file of your **panda_gazebo** package:

    *  gazebo_plugins
    *  gazebo_ros_control

*  Make a catkin package called **panda_control** in the src folder of you ros_ws. Once in your panda_control package,

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

*  In your **panda_gazebo** package

    *  Add another run_depend in the **package.xml** file

        *  panda_control (the catkin package you just made)

*  in the terminal: 
   
   .. code-block:: bash

      roslaunch panda_gazebo panda_positioncontrol_jointspace.launch

   *  you will be asked to enter a reference in joint space for the Panda arm and a finger displacement for the Panda hand
   *  enter these 7 joint angles and the two values for the finger displacement
   *  the robot will go to this reference configuration (maybe with some oscillations)


If this works, you can control the Panda robot by publishing joint angles (see your cpp file).

.. admonition:: todo

	Show video.

Effort control
^^^^^^^^^^^^^^^^

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




.. _Gazebo_Panda_Planning:

Planning with MoveIt
----------------------

.. admonition:: todo

	To write with updated GitHub repo. 


.. _Gazebo_Panda_ERG:

ERG
--------------------------

.. admonition:: todo

	To write with updated GitHub repo. 

.. _Gazebo_Panda_Planning_ERG:

Planning + ERG
--------------------------

.. admonition:: todo

	To write with updated GitHub repo. 