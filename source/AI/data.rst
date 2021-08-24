.. _Data:

Saving the data
===============

.. role:: raw-html(raw)
    :format: html

Here we will explain how to save the data you need in order to train the AI.

Hardware and software needed
----------------------------

For saving the data, you will need :
- A computer with an Nvidia GPU (the Jetson must be ok)
- A ZED 2 camera (need to be plug-in a computer with Nvidia)
- The software labelCloud to label data
- The github with all the files
- The frustrum-pointnet software
 

Time from A to Z
----------------

The labeling part takes 2h20 for 50 images per person. 
The training part takes 4 hours for 8000 images

How to save images with the camera
----------------------------------

Advice: 

Before starting to save your data with the ZED2, use a tripod to position the camera.
Put the camera at least 50cm from where you are going to take the picture. 
The camera has to be in an horizontal direction during the whole process (try not to lean the camera when you take pictures of the objects for easier labeling).
Between two picture move the camera slightly (by 5cm or 10cm).
During the process DO NOT MOVE YOUR OBJECTS OR IT WILL BE HARDER FOR LABELING.

To save the data with the ZED 2 camera, you have to clone the perception_pcl from `the github repository <https://github.com/A-Kouassi/labelYourCube.git>`_ which contain the rgb_pointcloud_saver code needed to 
Put it on your catkin_ws.

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone https://github.com/A-Kouassi/labelYourCube.git
    cd ..
    catkin_make

In order to launch the file, execute these two lines in two different terminals. In the first terminal you will have to launch the zed_wrapper

.. code-block:: bash

    roslaunch zed_wrapper zed2.launch
    rosrun pcl_ros rgb_pointcloud_saver number_of_the_first_image_to_take number_of_images_to_take (change the name of the package)

Every time you press enter, the program will take a new picture, display how many picture you took. 
Do not forget to move the camera between two pictures, but when taking a picture do not move the camera or the picture will be blurry.



Different files saved
---------------------

The program will save different files :
- The left and right RGB images save in rgb/left/ and rgb/right/
- The pointcloud save in pointclouds/
- The transformation between the next image and the actual one. For the first image there is no transformation. The transformation represents the movement of the camera between 2 images. The transformation 000000.json is the transormation between image 000000 and image 000001, and so on.

Label the data
--------------

(config.ini to do)

In order to label the data, you need labelCloud.
Launch labelCloud :

.. code-block:: bash

    python3 labelCloud.py

Now label the objects of the first pointcloud. Do not forget to change the name of the object you label on the right of the software. Press "save label" on the bottom left of the software

Open an other terminal, and execute :

.. code-block:: bash

    python3 deduce_label.py number_of_the_image_you_labeled (0 for the first image)

Now click next on the upper left corner, and you will see the next pointcloud with the predicted bounding box using the camera movement. So now you can adjust the bounding box.

Repeat this process for every pointcloud.

You will now obtain files in the labels/ folder

Convert data for training
-------------------------

Now that you have all the data needed for training, we need to convert them and put them into the good folder. So we created a little script for you to use to convert your data.

- kouassi.py -> label
- calib.py -> calib
- convert.py -> pcd to bin
- det.py -> crée rgb_2D