.. _Data:

Saving the data
===============

.. role:: raw-html(raw)
    :format: html

Here we will explain how to save the data you need in order to train the AI.

Hardware and software needed
----------------------------

.. _label:

*   For saving the data, you will need :
    *  A computer with an Nvidia GPU (the Jetson must be ok only for grabbing data!)
    *  A ZED 2 camera (need to be plug-in a computer with Nvidia)
    *  The  `labelCloud <https://github.com/ch-sa/labelCloud.git>`_ tool 
    *  The  `Frustum PointNets <https://github.com/ch-sa/labelCloud.git>`_ software 


Time from A to Z
----------------

The labeling part takes 2h20 for 50 images per person (it is the time we spent on labeling our first 100 images). 
The training part takes 4 hours for 8000 images

How to save images with the camera
----------------------------------

Advice: 

Before starting to save your data with the ZED2, use a tripod to position the camera.
Put the camera at least 50cm from where you are going to take the picture. 
The camera has to be in an horizontal direction during the whole process (try not to lean the camera when you take pictures of the objects for easier labeling).
Between two picture move the camera slightly (by 5cm or 10cm).
During the process DO NOT MOVE YOUR OBJECTS OR IT WILL BE HARDER FOR LABELING. 
Follow this step will reduce significantly the time spent on labeling.

To save the data with the ZED 2 camera, you have to clone the perception_pcl from this ` repository <https://github.com/A-Kouassi/perception_pcl.git>`_ which contain the rgb_pointcloud_saver code you will use yo save your data.

.. code-block:: bash

    cd path/to/catkin_ws/src
    git clone https://github.com/A-Kouassi/perception_pcl.git
    cd ..
    catkin_make

In order to launch the file, execute these two lines in two different terminals. In the first terminal you will have to launch the zed_wrapper. You can get it :ref:`here<wrapper>`.
But before that go to the zed wrapper configuration directory and open the common.yaml file

.. code-block:: bash

    cd path/to/catkin_ws/src/zed-ros-wrapper/zed_wrapper/params
    gedit common.yaml

At line 32 set ``self_calib`` to *false* to disable the camera calibration.
Now you can execute the wrapper

.. code-block:: bash

    roslaunch zed_wrapper zed2.launch

It is better if you save all the data in the labelCloud directory. To do this make sure you have cloned the :ref:`labelCloud<label>` repository.
Now open a second terminal and go in the labelCloud directory to run the rgb_pointcloud_saver code.

.. code-block:: bash

    cd path/to/labelCloud
    rosrun pcl_ros rgb_pointcloud_saver number_of_the_first_image_to_take number_of_images_to_take

Every time you press enter, the program will take a new picture and display how many picture you took. You can quit whenever you want by pressing q.

.. image:: ./images/rgb_pointcloud_0.png
    :width: 600


Do not forget to move the camera between two pictures, but when taking a picture do not move the camera or the picture will be blurry.

Different files saved
---------------------

The program will save different files :

*   The left and right RGB images are saved in rgb/left/ and rgb/right/ as .png files
*   The pointcloud are saved pointclouds/ as .pcd file
*   The transformation between two consecutive images. For the first image there is no transformation. The transformation represents the movement of the camera between 2 images. The transformation 000000.json is the transormation between image 000000 and image 000001, and so on.

Label the data
--------------

You can configure your keyboard and mouse setting by editing the config.ini. This will help you to be more efficient while labeling your pointcloud.

.. code-block:: bash

    cd path/to/labelCloud
    gedit config.ini

Here a :download:`configuration file<doc/config.ini>` we used.
    
Now you can label your data. For this go in your labelCloud directory and run the labelCloud.py code. 

.. code-block:: bash

    cd path/to/labelCloud
    python3 labelCloud.py
   
Make sure to perfectly label the object you want to detect in the pointcloud (do your best).
Do not forget to change the name of the object you label on the right of the software. Press "save label" on the bottom left of the software once you finished to label the object.

.. image:: ./images/first_label_pointcloud.png
    :width: 600
 
Now in an other terminal go in your labelCloud directory and run the deduce_label.py code.
This code will deduce the position of the future bounding box based on the labels from the previous pointcloud and the transformation file (this transformation file contains the translation and the rotation made by the camera between two pictures). 

.. code-block:: bash

    cd path/to/labelCloud
    python2 deduce_label.py number_of_the_image_you_labeled 0 #(0 for the first image) precise the current pointcloud

.. image:: ./images/deduce_labels_0.png
    :width: 600

Now click next on the upper left corner, and you will see the next pointcloud with the predicted bounding box using the camera movement. So now you can adjust the bounding box.
Repeat this process for every pointcloud.

.. raw:: html

    <video width="720" height="480" controls>
        <source src="../../../source/AI/videos/vid_label.mp4" type="video/mp4">
    Your browser does not support the video tag.
    </video>

Convert data for training
-------------------------

Now that you have all the data needed for training, we need to convert them and put them into the frustum pointnets directory . So we created a little script for you to use to convert your data. You can get everything from this `github repository <https://github.com/A-Kouassi/3d-object-detection.git>`_. This repository also include the frustum pointnets software.
Here the link to the `frustum pointnets  <https://github.com/charlesq34/frustum-pointnets.git>`_ github repository if you want to take a look at it.

Now that you have clone this repository, go in the convert directory execute the convert.sh script. This script will convert in the right format every files needed for the AI trining.

.. code-block:: bash

    cd path/to/3d-object-detection/frustum-pointnets-master/convert
    bash convert.sh path/to/labelCloud ../dataset/KITTI 

* This script will generate:
    *   the calibration files
    *   convert pcd to bin
    *   convert label
    *   copy the rgb images
    *   image_sets files
    *   create rgb_detection files


Changing files in frustum-pointnets
------------------------------------

If you want to train a custom model, you have to change some files from the frustum directories
(mettre tous les fichiers qui sont à changer et les lignes. Pour savoir lesquels faire il faut remettre la database kitti avec moi de files 200 pour voir si c'est comme nous à peu prè, et surtout il faut copier le code comme il est là pour les cubes)

Train the neural network
------------------------

Now that everything is set up, we will train our AI.
As it is explained in the Readme of frustum-pointnets, execute the following commands :

.. code-block:: bash

    sh scripts/command_prep_data.sh
    CUDA_VISIBLE_DEVICES=0 sh scripts/command_train_v1.sh

You will see a window ike this one appear :

(photo)

You want to maximise the IoU (which represents the the intersection divided by the union of the label and the predicted bounding box from the AI). It varies from 0 to 1. If it is 0, the two boxes do not overlap. If it is 1, the two boxes exactly match.


