Try the ZED SDK on the NVIDIA Jetson
====================================

.. _Try_the_zed:

Installing requirements
-----------------------
.. _run_python_script:

* install python3:
    * check if python3 is installed:

    .. code-block:: bash

        python3 --version

    * if python3 is not installed: follow `this tuturial <https://docs.python-guide.org/starting/install3/linux/>`_ to install it
    * check if pip is installed:

    .. code-block:: bash

        pip3 --version  # check if it is installed
        sudo apt-get install python3-pip  # install pip3 if it is not installed

* download ZED SDK
    * download `ZED SDK for Jetpack 4.5 <https://download.stereolabs.com/zedsdk/3.5/jp45/jetsons>`_. Here are `all the versions of ZED SDK <https://www.stereolabs.com/developers/release/>`_.
    * once the download is completed:

    .. code-block:: bash

        cd ~/Downloads  # path where the SDK is downloaded
        chmod +x ZED_SDK_Tegra_JP45_v3.5.0.run  # add execution permission
        ./ZED_SDK_Tegra_JP45_v3.5.0.run -- silent  # install in silent mode

    * check if ZED SDK is installed:
        * connect the camera to a USB port on the Jetson
        * run the commands:

        .. code-block:: bash

            cd /usr/local/zed/tools/
            ./ZED_Explorer

        * if everything goes well, we can see on the screen the images captured by the camera:

    .. image:: ./images/zed_explo.png
        :width: 600
        :alt: ZED_Explorer

Testing code
------------
Test if c++ and python code are working with the zed2 camera

* download example code
    clone the exemple code:

    .. code-block:: bash

        git clone https://github.com/stereolabs/zed-examples.git

    NB: the git clone command did not work for us at first, we had to reinstall git

    .. code-block:: bash

        sudo apt-get update
        sudo apt-get remove git
        sudo apt-get install git

* make c++ executable:
    * install cmake:

    .. code-block:: bash

        sudo apt-get update
        sudo apt-get install build-essential cmake

    * build application

    | Inside the zed_example directory there are several projects that can be build
    | Here is the example to build the camera control (you can build another executable the same way)

    .. code-block:: bash

        cd "path_to_zed-example/zed-example/camera control/cpp"
        mkdir build  # make build directory
        cd build  # go to build directory
        cmake .. #  generate project (
        make # compile application

    * run executable

    | The executable is normally in the build directory
    | Run the code below when you are in the directory where is the executable

    .. code-block:: bash

        ls  # check if the app is build
        ./ZED_Camera_Control

    * if everything goes well the application start

    .. image:: ./images/zed_cam_control.png
        :width: 600

* run python script:
    * Here is an example to run camera control script (you can run other scripts the same way):

    .. code-block:: bash

        cd "path_to_zed-example/zed-example/camera control/python"
        python3 camera_control.py

    * NB: we had to add "export OPENBLAS_CORETYPE=ARMV8" in the .bashrc file to avoid core dumped error

    .. code-block:: bash

        echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc

    * if a module is missing try to install it with pip3

    .. code-block:: bash

        pip3 install "module name"  # replace "module name" with the modole you want to install

* tutorial
    You can try `some tutorials to familiarise with the ZED SDK <https://www.stereolabs.com/docs/tutorials/>`_