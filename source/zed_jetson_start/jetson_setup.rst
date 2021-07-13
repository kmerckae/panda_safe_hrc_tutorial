.. _Get_Started:

Getting started with the NVIDIA Jetson Xavier NX
================================================

.. role:: raw-html(raw)
    :format: html

.. _What_do_you_need_Jetson:

What do you need?
-----------------
*  A Jetson Xavier NX Developer Kit 

    *  Jetson Xavier NX module (P3668-0000) with heatsink
    *  Reference carrier board (P3509-0000)
    *  19V power supply
    *  A small paper card with quick start and support information

*  An empty microSD card (16GB UHS-1 minimum)
*  A USB keyboard
*  A USB mouse
*  A computer display (either HDMI or DP)

In case the microSD card you want to use for this project is not empty, then format it first. 
When your microSD card has multiple partitions, then you first have to repartition the microSD card. 
Therefore, you can use DiskPart which is included on Windows. 
See |repartition_sdcard_link| for more information. 
After you have repartitioned the microSD card, you have to format it before usage.  

.. |repartition_sdcard_link| raw:: html

   <a href="https://www.instructables.com/Repartition-SD-Card-Windows/" target="_blank">Repartition-SD-Card-in-Windows</a>


.. _setup_and_boot_JetsonXavierNX:

Set up and boot the Jetson Xavier NX
------------------------------------
Follow the |get_started_jetson_xavier_nx_devkit_link| tutorial from NVIDIA.
Below you will find some clarifications and solutions to problems that can occur when following the NVIDIA tutorial. 

.. |get_started_jetson_xavier_nx_devkit_link| raw:: html

   <a href="https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit" target="_blank">Getting Started With Jetson Xavier NX Developer Kit</a>

Write Image to the microSD Card
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you use Etcher on Windows to write the SD card image, it is possible that you will not see 
"Select image" as in the picture of the NVIDIA tutorial, but "select file". 
This is the same, so in that first step, click on "select file" and choose your *.img.zip* file. 

Afterwards, when you select the drive in Etcher, you can get the message that the SD card is locked. 
Usually that means that the SD card is physically locked and that you have to |unlock_sd_card|. 

.. |unlock_sd_card| raw:: html

   <a href="https://forum.dexterindustries.com/t/solved-etcher-says-sd-card-is-locked/2143" target="_blank">physically unlock the SD card</a>


Setup and First Boot
^^^^^^^^^^^^^^^^^^^^

When you are asked to choose the **APP Partition Size**, it is recommended to choose the maximum accepted size. 

When you select the **Nvpmodel Mode**, keep at the beginning the default settings, which is MODE_10W_DESKTOP - (Default). 
Refer to |NVIDIA_Jetson_Linux_Developer_Guide| for further information. 

.. |NVIDIA_Jetson_Linux_Developer_Guide| raw:: html

   <a href="https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0E0VO0HA" target="_blank">NVIDIA Jetson Linux Developer Guide</a>


The **username** and **password** we have used to log in are:

* username: xavier1 / xavier2 
* password: JetsonXavier

Normally you can automatically connect to a Wireless Network. 



Avoid crashing and latency issues
---------------------------------
You can avoid crashing and latency issues with the NVIDIA Jetson Xavier by switching 
the power mode supply of the NVIDIA Jetson to "MODE 15W 6CORE", as depicted in the figure below.

.. image:: ./images/power_mode.png
    :width: 300

In order to reduce lags on the Jetson turn on the fan to 100%, by doing the following steps. 

* Install jetson-stats:

.. code:: bash

    git clone https://github.com/rbonghi/jetson_stats
    sudo apt-get update
    sudo apt-get install python3-pip  # install pip3
    cd jetson_stats
    sudo -H pip3 install -U jetson-stats

* Restart your Jetson and run :

.. code:: bash

    jtop  # start jtop

* The following window will open with all the information of the Jetson

.. image:: ./images/jtop.png
    :width: 600

* Navigate with the arrow keys to tab 5CTRL. 
  You can set the fan method to manual by clicking with your mouse on "manual". 
  In the manual fan mode, you cna increase the fan speed by pressing the "p" key and decrease the fan speed by pressing the "m" key. 

.. image:: ./images/jtop_fan.png
    :width: 600


.. note:: 
   Note: if you try to install something and the Jetson keeps shutting down, you can :ref:`set the Jetson in shell mode<jetson_shell_mode>`.
   :raw-html:`<font color="Blue"> Kelly still needs to check this.   </font>`



Network Connection problems
---------------------------
Although we didn't have this problem with the NVIDIA Jetson Xavier NX, 
we couldn't connect automatically to WiFi with the NVIDIA Jetson Nano. 
As an inspiration to solve connection problems with a Jetson, we added the explanation for the |NVIDIA_Jetson_Nano_tutorial|. 

.. |NVIDIA_Jetson_Nano_tutorial| raw:: html

   <a href="https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit" target="_blank">NVIDIA Jetson Nano</a>

WiFi Connection
^^^^^^^^^^^^^^^
Use a |WiFi_USB_adapter| to connect to WiFi. 
If that doesn't work, check this |WiFi_USB_adapter_problem| and its |WiFi_USB_adapter_solution|. 

.. |WiFi_USB_adapter| raw:: html

   <a href="https://learn.sparkfun.com/tutorials/adding-wifi-to-the-nvidia-jetson/all#hardware-overview-and-assembly" target="_blank">WiFi USB adapter</a>

.. |WiFi_USB_adapter_problem| raw:: html

   <a href="https://forums.developer.nvidia.com/t/jetson-nano-wifi-usb-adapter/73157" target="_blank">problem</a>

.. |WiFi_USB_adapter_solution| raw:: html

   <a href="https://forums.developer.nvidia.com/t/jetson-nano-wifi/72269" target="_blank">solution</a>


Ethernet Connection
^^^^^^^^^^^^^^^^^^^   
*  Go in the system parameters then in network and find the wired option with a connection.
*  In the options of the wired connection, go to ipv4, switch the method to manual and enter the IP, submask and DNS servers.  
*  for internet connection : IP=134.184.20.129, subnet mask=24 DNS= 134.184.250.7, 134.184.15.13, Gateway=134.184.20.100
*  The jetson will also be connected directly to the main computer( franka computer). For this local connection, no DNS is needed. 
   The only constrain is that the last digits of the IP adress must be unique on the local netwwork. 
   For the rest of the tutorial the local IP will be 192.168.4.66, subnet=24. 
   The master has the IP 192.168.4.65 and will be used as gateway for the direct connection.  


SSH Connection
^^^^^^^^^^^^^^
*  Install openssh-server on the Jetson Nano. (This one is the server and doesn't need the client package). 
*  Install openssh-client and openssh-server on the Franka desktop. This computer will connect to the jetson so we need to authorize the ssh access. 
*  The parameters to modify on the Jetson are in the file:

    .. code:: bash

        sudo gedit /etc/ssh/sshd_config 

*  Find the parameter *PermitRootLogin*, uncomment the line and change the value to : yes. 
*  add the line : *AllowUsers root jetson-nano ( root + username)*. These changes allows to run administrator commands.
*  Change *PasswordAuthentication* to yes. This allows to use a password to connect. 
   For devices on internet and not on a local connection only, a key is reccomened. 
*  Restart ssh: 

    .. code:: bash

        sudo service ssh restart