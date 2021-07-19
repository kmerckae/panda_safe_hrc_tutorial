Panda arm and Octomap mapping
=============================

In this section, we will see how to display in real time the octomap and the point cloud into RViz with the panda arm.
Before starting make sure you have done the :ref:`transformation between the ZED2 and the robot frame <>`.

Note: Once the octomap will be generated we will have to remove the panda arm from the octomap otherwise it will be recognized as an obstacle.

Point cloud and Panda arm
*************************

Octomap and Panda arm
*********************

Remove the octomap of the robot from the world
**********************************************

To generate the octomap without the robot arm, you will need to modify the grab_detected_object_test.launch file by adding this:

.. code:: XML

  <node pkg="tf2_ros" type="static_transform_publisher" name="to_panda" args="0 0 0 0 0 0  world panda_link0" />
  <node pkg="tf2_ros" type="static_transform_publisher" name="world_link_to_map" 
    args="$(arg transfo_world2map) world map" />

  <param name="octomap_frame" type="string" value="map" />
  <param name="octomap_resolution" type="double" value="0.03" />
  <param name="max_range" type="double" value="3.0" />
  <rosparam command="load" file="$(find panda_moveit_config)/config/sensors_kinect_pointcloud.yaml"/>

.. image:: ./images/octo_without_arm.png
  :width: 300

Note: If there are still residuals of the robot arm left, it is because the transformation between the camera and the robot frame is not perfect.
