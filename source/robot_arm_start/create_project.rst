

=============
Add a project
=============

.. _Download_project:

Downloading an existing project
===============================

You can download an existing project and use the controllers that are contained. You can use `the project we made during our internship <https://github.com/panda-brubotics/franka_constrained_control>`_ which contains (our work, for now it's empty).

Once you downloaded it, go to ``libfranka`` and build it :

.. code-block:: bash

   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   cmake --build .

.. code-block:: bash

   cd ../../catkin_ws/src/franka_ros
   catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

.. warning::

   The path of ``-DFranka_DIR:PATH`` MUST be an absolute path to the libfranka's build directory.

Now the project is build, and you will use ``catkin_make`` to build in the continuation.

.. _Create_project:

Create our own project
======================

.. _Create_repository :

Create our own repository
*************************

This section is normally not very useful because all the packages are already installed on the computer next to the arm. But if you want to create your own repository, follow this !

Let's create our own repository. We will install ``libfranka`` and ``franka_ros`` in our repository. The first thing to do is to install ``libfranka``, because ``franka_ros`` use it. This tutorial is inspired from `the tutorial Kelly Merckaert did <https://github.com/kmerckae/franka_constrained_control>`_ and from `the one from franka emika website <https://frankaemika.github.io/docs/installation_linux.html>`_

First of all make sure that ros is installed :

.. code-block:: bash

   sudo apt install ros-melodic-libfranka ros-melodic-franka-ros

Before building from source, please uninstall existing installations of ``libfranka`` and ``franka_ros`` to avoid conflicts:

.. code-block:: bash

   sudo apt remove "*libfranka*"

Create a directory (or a git directory if you want to make a git repository) before installing ``libfranka`` and ``franka_ros``.

.. _Install_libfranka :

Install libfranka
^^^^^^^^^^^^^^^^^

To build libfranka, install the following dependencies from Ubuntu’s package manager:

.. code-block:: bash

   sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Then, download the source code by cloning libfranka from `libfranka's GitHub <https://github.com/frankaemika/libfranka>`_ in your directory (or git repository):

.. code-block:: bash

   git clone --recursive https://github.com/frankaemika/libfranka
   cd libfranka

By default, this will check out the newest release of ``libfranka``. But we want to use the version *0.7.1*, so we have to change the branch :

.. code-block:: bash

   git checkout 0.7.1
   git submodule update

In the source directory, create a build directory and run CMake:

.. code-block:: bash

   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   cmake --build .

Try to launch files in the ``build/exemples`` directory to see if the installation is completed, like this :

.. code-block:: bash

   ./echo_robot_state 192.168.2.106
   ./print_joint_poses 192.168.2.106

.. _Install_franka_ros :

Install franka_ros
^^^^^^^^^^^^^^^^^^

Now that you installed ``libfranka``, we will install ``franka_ros``.

To do so, go to your directory (or git repository) and then create a Catkin workspace :

.. code-block:: bash

   mkdir -p catkin_ws/src
   cd catkin_ws
   source /opt/ros/melodic/setup.sh
   catkin_init_workspace src

Then clone the ``franka_ros`` repository from `franka_ros's GitHub <https://github.com/frankaemika/franka_ros>`_:

.. code-block:: bash

   git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

By default, this will check out the newest release of ``franka_ros``. But we want to use the version *0.6.0*, so we have to change the branch :

.. code-block:: bash

   cd src/franka_ros
   git checkout 0.6.0

Install any missing dependencies and build the packages :

.. code-block:: bash

   rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
   catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
   source devel/setup.sh

.. warning::

   The path of ``-DFranka_DIR:PATH`` MUST be an absolute path to the libfranka's build directory.

remove submodules

.. _Create_controller :

Create our own controller
*************************

Now that you :ref:`tested example controllers<Command_control>`, it is time to create your own controller in order to control the arm !

To do so, follow this `link <https://www.franka-community.de/t/starting-to-write-a-new-controller/1537>`_.

After this tutorial, you should be able to create your own controller.
