.. _Robot_Network:

Set Up the Robot Network
========================

.. role:: raw-html(raw)
    :format: html

Hardware connections
--------------------

In the figure below you can see the official hardware connection documentation from Franka Emika
which you can find in the :download:`Franka Emika Panda manual  <doc/FrankaPandaManual.pdf>`. 

.. image:: ./images/panda-set-up.png
    :align: center
    :width: 700px

In our setup we have the following connections:

*  desktop to Panda Arm with ethernet cable

*  desktop to Panda Control with ethernet cable

*  Panda Arm to Panda Control with connection cable

*  external activation device to Panda Arm

*  Panda Control to Panda Arm with earth cable

*  Panda Control to power supply

*  dekstop to an ethernet port with an ethernet cable (if there is no Wi-Fi)


Set up the network connections
------------------------------

Turn on the Panda Control via the On-Off switch at the back of the Panda Control.

After the Control has been switched on, or supplied with energy, Panda boots up.
In this process, the safety locking system is activated, movements are thus locked mechanically.
The display lights on the base and the pilot **flash yellow**.

As soon as Panda has booted up, the display lights are lit **yellow continuously**.


.. note:: The part below is specifically written for the use of robot *Panda 2* in the R&MM lab at Vrije Universiteit Brussel
          with the desktop that is already set up to work with this robot. 
          This robot has shop floor IP address 192.168.2.106 and Netmask 255.255.255.0. 
          
          .. image:: ./images/Panda2_shopfloor_network.png
            :width: 350px

          For the general version of this documentation, we refer to |frankaemika-setting-up-robot-network| for the Franka Emika Panda robot.

.. |frankaemika-setting-up-robot-network| raw:: html

    <a href="https://frankaemika.github.io/docs/getting_started.html#setting-up-the-network" target="_blank">setting up the network</a>

In case there is no Wi-Fi connection, you should see three ethernet connections:

*  a connection to ``FrankaControl``

   .. image:: ./images/connection_FrankaControl.png
    :width: 450px

*  a connection to ``FrankaArm``

   .. image:: ./images/connection_FrankaArm.png
    :width: 450px

*  a connection to ``Internet connection``

   Note that this is the personal static IP address that is assigned to Kelly Merckaert at Vrije Universiteit Brussel. 
   At VUB, they work with static IP addresses, so if you need internet via an ethernet connection, you have to ask for your own IP address. 

   .. image:: ./images/connection_internet.png
    :width: 450px

Depending on which ethernet ports you have connected, you have to select the right ethernet connections.
In the example below we have connected port enp10s0f0 to ``FrankaControl``,
port enp10s0f1 to ``FrankaArm``, and port enp12s0 to ``Internet connection``.

   .. image:: ./images/ethernetport_FrankaControl.png
    :align: left
    :width: 200px

   .. image:: ./images/ethernetport_FrankaArm.png
    :align: left
    :width: 200px

   .. image:: ./images/ethernetport_internet.png
    :align: left
    :width: 192px