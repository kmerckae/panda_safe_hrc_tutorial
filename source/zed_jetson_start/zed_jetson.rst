.. _Zed_Jetson:

Getting Started with ZED SDK on Jetson
=======================================

.. role:: raw-html(raw)
    :format: html
    
.. note:: This page is generally written.

What do you need?
-----------------
On top of the things listed in :ref:`getting started with NVIDIA Jetson Xavier NX<Jetson_Setup>`: 

*  ZED 2 camera box 

    *  ZED 2 Stereo camera
    *  1.2m built-in USB 3.0 cable
    *  mini-tripod stand

    .. _run_python_script:

*  python3 should be installed

    *  check that python3 is installed by running the following command in the terminal

       .. code-block:: bash

            python3 --version

    *  if python3 is not yet installed, then follow this |python3_installation_tutorial|. 

        .. |python3_installation_tutorial| raw:: html

            <a href="https://docs.python-guide.org/starting/install3/linux/" target="_blank">installation tutorial</a>

*  pip should be installed

    *  check that pip is installed by running the following command in the terminal

       .. code-block:: bash

            pip3 --version 

    *  if pip is not yet installed, then run the command

       .. code-block:: bash

            sudo apt-get install python3-pip 

Download and install the ZED SDK
--------------------------------
Go to |Download_Install_ZED_SDK| and download the latest |ZED_SDK_Jetpack| (4.5 when writing the tutorial).

.. |Download_Install_ZED_SDK| raw:: html

    <a href="https://www.stereolabs.com/docs/installation/jetson/#download-and-install-the-zed-sdk" target="_blank">Download and Install the ZED SDK</a>

.. |ZED_SDK_Jetpack| raw:: html

    <a href="https://www.stereolabs.com/developers/release/" target="_blank">ZED SDK for Jetpack</a>

Once the download is completed, do

.. code-block:: bash

    cd /Downloads  # path where the SDK is downloaded
    chmod +x ZED_SDK_Tegra_JP45_v3.5.0.run  # add execution permission
    ./ZED_SDK_Tegra_JP45_v3.5.0.run -- silent  # install in silent mode


Check if the ZED SDK is properly installed. 

*  Connect the ZED 2 camera to a USB port on the Jetson. 
*  Run the commands: 
   
   .. code-block:: bash

        cd /usr/local/zed/tools/
        ./ZED_Explorer

*  If the ZED SDK is installed well, we should see the images capture by the camera on the screen: 

   .. image:: ./images/zed_explo.png
        :width: 600
        :alt: ZED_Explorer

.. note:: 
    |ZED_SDK_Jetson_Installation| writes that you first need to install JetPack. 
    However, JetPack is already installed when you flashed the SD card as in :ref:`Set Up and Boot the Jetson Xavier NX<Jetson_Setup>`. 

.. |ZED_SDK_Jetson_Installation| raw:: html

    <a href="https://www.stereolabs.com/docs/installation/jetson/" target="_blank">ZED SDK installation on Nvidia Jetson</a>