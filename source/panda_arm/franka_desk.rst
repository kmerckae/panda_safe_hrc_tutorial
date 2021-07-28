.. _Franka_Desk:

Start and Shut Down the Robot via Franka Desk
=============================================

.. role:: raw-html(raw)
    :format: html
    
.. note:: This page is generally written.
          You only have to change the IP address of the robot if you want to follow this page for a different Panda robot. 


Start Franka Desk
-----------------

Once the Panda has booted up, you can start Franka Desk. 
Open your web browser and type ``192.168.2.106`` in the search bar. 
When the robot is well connected, you should obtain the following window:

.. image:: ./images/franka_desk.png
    :align: center
    :width: 700px

Settings
--------

:raw-html:`<font color="red"> You can attach and detach the end-effector. 
If you detach the end-effector and want to attach another end-effector, you can add the mass and inertia in the settings window.
Add screenshot of this settings window.  </font>`

Unlock the joints
-----------------

At this point, the display lights of the Panda Arm should be lit **yellow continuously**.
This means that the Panda has booted up and that the safety locking system is activated, whereby movements are locked mechanically. 

Make sure that the external activation device is in the closed state (pressed down). 

The safety locking system can now be opened with the button ``click to unlock joints`` in the sidebar of Franka Desk. 

.. image:: ./images/unlock.png
    :align: center
    :width: 700px

After you have clicked on ``open``, you should hear seven clicks of the seven joints that are being unlocked. 

The display lights should now be lit **white continuously**. 
Desks’s sidebar shows now ``joints unlocked``. 
Panda is now in the monitored stop state.

Shut down the robot 
-------------------

When you are done using the robot, don't forget to put the external activation device in the closed state (pressed down) 
and lock the joints before shutting down the Franka Control and the desktop. 

The safety locking system can be closed with the button ``click to lock joints`` in the sidebar of Franka Desk. 
You will here one click which means that all the seven robot joints are locked and afterwards the display lights should be lit **yellow continuously**. 