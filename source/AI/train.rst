.. _Train:

Training the AI
===============

.. role:: raw-html(raw)
    :format: html

Here we will explain how to train the neural network.

Train the neural network
------------------------

Now that everything is set up, we will train our AI.
As it is explained in the Readme of frustum-pointnets, execute the following commands :

.. code-block:: bash

    sh scripts/command_prep_data.sh
    CUDA_VISIBLE_DEVICES=0 sh scripts/command_train_v1.sh

You will see a window like this one appear :

.. image:: ./images/train.png
    :width: 300
    :align: center

The training will try to reduce the mean loss. The loss represents the error between the prediction and the label. So the lower it is, the better it is. At a certain moment, you will see that the loss does not move : the algorithm can't learn more, you can stop it.

Also, take a look at the IoU (which represents the the intersection divided by the union of the label and the predicted bounding box from the AI). It varies from 0 to 1. If it is 0, the two boxes do not overlap. If it is 1, the two boxes exactly match.

.. image:: ./images/iou.png
    :width: 300
    :align: center

Also, if you want to upgrade the number of data, you can update the convert.sh script by taking in account the right images, so you can double the data you have

Test the neural network
-----------------------

Now, you just try to predict the labels :

.. code-block:: bash

    sh scripts/command_test_v1.sh

The results of this command will be stored in the train/detection_results_v1/data/ folder
You can visualize the results by swapping the labels files in the dataset/KITTI/object/training/label_2/ folder with those in the train/detection_results_v1/data/ folder.
Then, execute :

.. code-block:: bash

    python3 scripts/kitti_object.py

You can see if the network performs well on the data it trained on

.. image:: ./images/first_result.png
    :width: 300
    :align: center

.. image:: ./images/second_result.png
    :width: 300
    :align: center
