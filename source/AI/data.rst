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

To save the data with the ZED 2 camera, you have to clone the perception_pcl from the `github repository <https://github.com/A-Kouassi/perception_pcl.git>`_ which contain the rgb_pointcloud_saver code you will use yo save your data.

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
    
In order to label the data, you need labelCloud.
Launch labelCloud :

.. code-block:: bash

    cd path/to/labelCloud
    python3 labelCloud.py

.. image:: ./images/first_label_pointcloud.png
    :width: 600


Now label the objects of the first pointcloud. Do not forget to change the name of the object you label on the right of the software. Press "save label" on the bottom left of the software

Open an other terminal, and execute :

.. code-block:: bash

    python3 deduce_label.py number_of_the_image_you_labeled (0 for the first image)

Now click next on the upper left corner, and you will see the next pointcloud with the predicted bounding box using the camera movement. So now you can adjust the bounding box.

Repeat this process for every pointcloud.

You will now obtain files in the labels/ folder

Convert data for training
-------------------------

Now that you have all the data needed for training, we need to convert them and put them into the good folder. So we created a little script for you to use to convert your data. You can download this github repository in order to do so. Put the file in the frustrum-pointnets directory.
Now, execute in the convert folder :

.. code-block:: bash

    bash convert.sh path_label_cloud path_database (ex : ../dataset/KITTI for path_database)

Changing files in frustrum-pointnets
------------------------------------

If you want to train a custom model, you have to change some files from the frustrum directories
(mettre tous les fichiers qui sont à changer et les lignes. Pour savoir lesquels faire il faut remettre la database kitti avec moi de files 200 pour voir si c'est comme nous à peu prè, et surtout il faut copier le code comme il est là pour les cubes)

Train the neural network
------------------------

Now that everything is set up, 