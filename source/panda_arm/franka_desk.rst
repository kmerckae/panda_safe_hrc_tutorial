.. _Franka_Desk:

Start and Shut Down the Robot via Franka Desk
=============================================

.. role:: raw-html(raw)
    :format: html
    
.. note:: To follow this page for another robot that the *Panda 2* in the R&MM lab at Vrije Universiteit Brussel, 
          you will have to change the IP address of the robot and the username and password you will need to enter Franka Desk. 


Start Franka Desk
-----------------

Once the Panda has booted up, you can start Franka Desk. 
Open your web browser and type ``192.168.2.106`` in the search bar. 
You will see a window where you will have to give your username and password to enter Franka Desk. 
The username is *admin* and the password is *franka123*. 

.. image:: ./images/franka_desk_login.png
    :align: center
    :width: 400px


When the robot is well connected and you have signed in, you should obtain the following window:

.. image:: ./images/franka_desk.png
    :align: center
    :width: 700px

Settings
--------

You can go to the settings window in Franka Desk by clicking on the two lines in the right upper corner. 
Like that you can go to the settings menu, but you can also download the User Manual and the Cobot Pump Manual. 

   .. image:: ./images/FrankaDesk_Settings.png
    :align: center
    :width: 250px
    

In the settings menu, you can see in the **Dashboard** tab the system, robot, and network settings. 
In the **Network** tab, you can change the robot and the shop floor network. 
In the **System** tab, you can install features like the Franka Control Interface, do a system update, download log files, etc. 


In the **End-Effector** tab, you can select the standard ``Franka Hand`` or select ``Other``. 

*  If you are using the standard Franka Hand end-effector, then select the ``Franka Hand``. 
   In case the Franka Hand is *uninitialized* in the Dashboard tab, you can re-initialize the hand by clicking on ``HOMING``. 
   Afterwards the hand should be *initialized* in the Dashboard tab. 

   .. image:: ./images/FrankaDesk_EndEffector_FrankaHand.png
    :width: 700px

*  If you detach the Franka Hand end-effector and want to attach another end-effector, then you have to select ``Other``. 
   Like that, you will be able to give the properties of your new end-effector, as in the example below. 

   .. image:: ./images/FrankaDesk_EndEffector_Other.png
    :width: 700px

To go back to the **Desk**, click again on the lines in the upper right corner and click on Desk. 


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