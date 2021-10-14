.. _Gazebo_Panda_Visualization:

Panda Visualization
======================================

.. role:: raw-html(raw)
    :format: html

.. admonition:: todo

	Make updated GitHub repository with ROS Noetic on Ubuntu 20.04 and rewrite this part of the tutorial.


Create a catkin workspace
--------------------------

.. |atlassian_setting_up_a_repository| raw:: html

    <a href="https://www.atlassian.com/git/tutorials/setting-up-a-repository" target="_blank">set up a repository</a>

If you want to work with Git version control (which I highly recommend), then you first need to |atlassian_setting_up_a_repository|. 

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

Visualize the Panda robot in RViz
---------------------------------

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
-------------------------------------

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

.. note:: In :download:`Dynamic Identification of the Franka Emika Panda Robot with Retrieval of Feasible Parameters Using Penalty-Based Optimization <doc/PandaDynamicIdentification.pdf>`
          and :download:`its supplementary material <doc/PandaDynamicIdentification_SupplementaryMaterial.pdf>`, 
          the authors identified the dynamic parameters of the Panda robot. 
          I implemented them in my code some months ago, check the last version of panda_arm.xacro in panda_description.