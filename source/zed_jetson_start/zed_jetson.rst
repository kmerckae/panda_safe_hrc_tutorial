Use the ZED SDK on Jetson Xavier NX
====================================

.. role:: raw-html(raw)
    :format: html

.. _What_do_you_need_ZED_Jetson:

What do you need?
-----------------
On top of the things listed in :ref:`getting started with NVIDIA Jetson Xavier NX<setup_and_boot_JetsonXavierNX>`: 

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
    However, JetPack is already installed when you flashed the SD card as in :ref:`Set Up and Boot the Jetson Xavier NX<setup_and_boot_JetsonXavierNX>`. 

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

For the object detection (birds eye viewer) example, you have to press 'c' to clear filters
such that the program can detect objects from the available object classes.  
Otherwise, the program can only detect people and vehicles. 

    .. image:: ./images/zed-objectdetection-person-animal.png
        :width: 600

    .. image:: ./images/zed-objectdetection-animal-electronics.png
        :width: 600

Python sample code
^^^^^^^^^^^^^^^^^^
* To run the camera control script (you can run other scripts the same way):

    .. code-block:: bash

        cd "path_to_zed-examples/zed-examples/camera control/python"
        python3 camera_control.py

* If a module is missing, you will receive a ModuleNotFoundError. 
  Solve this by installing the module with pip3:

    .. code-block:: bash

        pip3 install "module name"  # replace "module name" with the modole you want to install

  If you installed a module by mistake, then you can uninstall it with pip3:

    .. code-block:: bash

        pip3 uninstall "module name"  # replace "module name" with the modole you want to uninstall

.. note:: 
    For the following modules you will have to install the required module with another name than mentioned in the ModuleNotFoundError. 
    At the left you see the name that is mentioned, at the right you see the name you have to use in the pip3 install. 

    *  |OpenGL| :raw-html:`&rarr;` |PyOpenGL| 

.. |OpenGL| raw:: html

    <a href="https://pypi.org/project/opengl/" target="_blank">OpenGL</a>

.. |PyOpenGL| raw:: html

    <a href="https://pypi.org/project/PyOpenGL/" target="_blank">PyOpenGL</a>



.. warning::
    We had a core dumped error when using the old SD card. 
    You can avoid a core dumped error by adding "export OPENBLAS_CORETYPE=ARMV8" in the .bashrc file.

    .. code-block:: bash

        echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc

ZED Tutorials
^^^^^^^^^^^^^
You can try some |ZED_Tutorials| to familiarise with the ZED SDK.
The C++ and Python version can be launched in the same way as explained above. 
In these tutorials information will be printed in the terminal. 


.. |ZED_Tutorials| raw:: html

    <a href="https://www.stereolabs.com/docs/tutorials/" target="_blank">tutorials</a>