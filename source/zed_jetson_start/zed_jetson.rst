Use the ZED SDK with the Jetson Xavier NX
=========================================

.. role:: raw-html(raw)
    :format: html

.. _Try_the_zed:

What do you need?
-----------------
On top of the things listed in :ref:`getting started with NVIDIA Jetson Xavier NX<Get_Started>`: 

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

.. _ZED_SDK:

Download and Install the ZED SDK
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

:raw-html:`<font color="Tomato"> I followed your steps and everything works fine, but I'm a bit confused. 
When I go to </font>` |ZED_SDK_Jetson_Installation| 
:raw-html:`<font color="Tomato"> it seems that I first have to install JetPack, but now I didn't do that, right? </font>`

.. |ZED_SDK_Jetson_Installation| raw:: html

    <a href="https://www.stereolabs.com/docs/installation/jetson/" target="_blank">ZED SDK installation on Nvidia Jetson</a>



Test the zed-examples 
----------------------
To test ZED's multiple |ZED_Code_Examples|, we first have to download the example code in a directory of your choice. 

.. |ZED_Code_Examples| raw:: html

    <a href="https://www.stereolabs.com/docs/code-samples/" target="_blank">code examples</a>


.. code-block:: bash

    git clone https://github.com/stereolabs/zed-examples.git

If the git clone command doesn't work anymore, then reinstall git:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get remove git
    sudo apt-get install git

C++ sample code
^^^^^^^^^^^^^^^

For the c++ examples, you will have to install cmake:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install build-essential cmake

Inside the **zed_examples** directory there are several c++ examples that can be build and run. 
Here we explain how to do this for the camera control example. 
For other examples, you have to do this in a similar way. 

*  build executable
    
    .. code-block:: bash

        cd "path_to_zed-examples/zed-examples/camera control/cpp"
        mkdir build  # make build directory
        cd build  # go to build directory
        cmake .. #  generate project (
        make # compile application

*  run executable (which is now in the build directory)

    .. code-block:: bash

        ls  # check if the app is build
        ./ZED_Camera_Control

*  If the example starts well, it should show the terminal in which camera settings can be modified and should display the resulting image.

    .. image:: ./images/zed_cam_control.png
        :width: 600

Python sample code
^^^^^^^^^^^^^^^^^^
* To run the camera control script (you can run other scripts the same way):

    .. code-block:: bash

        cd "path_to_zed-examples/zed-examples/camera control/python"
        python3 camera_control.py

* if a module is missing try to install it with pip3

    .. code-block:: bash

        pip3 install "module name"  # replace "module name" with the modole you want to install

:raw-html:`<font color="Tomato"> When I run the body tracking example in python, 
I get the ModuleNotFoundError : No modle named 'OpenGL'.
However, when I run pip3 install OpenGL, it says that it is successfully installed, 
but when I run again the body tracking example, it says again that there is no module name OpenGL.  </font>` 

To avoid a core dumped error, we had to add "export OPENBLAS_CORETYPE=ARMV8" in the .bashrc file.

.. code-block:: bash

    echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc

:raw-html:`<font color="Tomato"> Do you still have this problem with the new SD card? 
Or was the origin of the problem something else? 
I didn't get a core dumped error. </font>` 

Tutorials
^^^^^^^^^
You can try some |ZED_Tutorials| to familiarise with the ZED SDK.


.. |ZED_Tutorials| raw:: html

    <a href="https://www.stereolabs.com/docs/tutorials/" target="_blank">tutorials</a>