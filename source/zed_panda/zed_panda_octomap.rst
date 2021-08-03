Octomap for Panda arm
=====================

This section will help you to make and save an octomap of the panda arm environnement

Parameters for the octomap
--------------------------

Set parameters to filter the pointcloud
***************************************

In this section we will set the parameters used to filter the point cloud so we can create an octomap from the filtered point cloud
In the filterd point cloud, the robot is removed

* When you are in your ``catkin_ws`` directory go to ``src/franka_ros/panda_moveit_config/config`` 
* Open the file ``sensors_kinect_pointcloud.yaml``
* From this file, you can change some parameters (source : `Perception Pipeline Tutorial <http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/perception_pipeline/perception_pipeline_tutorial.html>`_):

  * The general parameters are:
  
    * *sensor_plugin:* The name of the plugin that we are using.
    * *max_update_rate:* The octomap representation will be updated at rate less than or equal to this value.

  * Parameters specific to the Point cloud updater are:
  
    * *point_cloud_topic:* This specifies the topic to listen on for a point cloud.
    * *max_range:* (in m) Points further than this will not be used.
    * *point_subsample:* Choose one of every point_subsample points.
    * *padding_offset:* The size of the padding (in cm).
    * *padding_scale:* The scale of the padding.
    * *filtered_cloud_topic:* The topic on which the filtered cloud will be published (mainly for debugging). The filtering cloud is the resultant cloud after self-filtering has been performed.



Set the octomap parameters:
***************************
  
* Go to the octomap_tools package
* Go to the launch directory
* Open the file ``save_panda_octomap.launch``
* From this file you can change some parameters (lines 22 to 25)
* The parameters are listed and explained `here in the section 2.2.4 <http://wiki.ros.org/octomap_server>`_

Create and save the octomap
---------------------------

* Go to your catkin_ws source your workspace and run the program

.. code-block:: bash

  cd path/to/your/workspace
  source devel/setup.sh
  roslaunch octomap_tools save_panda_octomap.launch

* In rviz load the config localized in the package ``zed_franka_planner`` in rviz/zed_panda_save_octomap.rviz
* Adjust the transformation between the camera and the robot by using the ``Static Transform Publisher`` in rviz
* Write down the transformation in quaternion
* Once you get a satisfying octomap, you can save it:

  * Go to the directory where you want to save the octomap
  * Open a terminal there
  * Run ``rosrun octomap_server octomap_saver -f panda_robot_octomap.bt``

Plan in the octomap environnement
---------------------------------
| Set your work space like this: 

.. image:: images/plan_in_octomap_workspace.jpg
  :width: 600

| Save an octomap of your workspace
| This program is a demo of planning in the octomap

* Go to your catkin_ws source your workspace and run the program

.. code-block:: bash

  cd path/to/your/workspace
  source devel/setup.sh
  roslaunch zed_franka_planner plan_in_octomap.launch path:=path/to/your/saved/octomap panda_to_map_transfo:="1.3 0.06 0.38 0 0 1 0"
  # "1.3 0.06 0.38 0 0 1 0" is the transformation in quaternion between the camera and the robot when you saved the octomap

* In rviz load the config ``plan_in_octomap.rviz`` (same directory as earlier)