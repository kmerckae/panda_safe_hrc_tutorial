.. _FCI_Project:

Start Franka Control Interface Project
======================================

.. role:: raw-html(raw)
    :format: html

.. note:: This page is generally written. 
          For the *Panda 2* robot in the R&MM lab at Vrije Universiteit Brussel we replace <fci-ip> with ``192.168.2.106``

The Franka Control Interface (FCI) allows a fast and direct low-level bidirectional connection to the Arm and Hand.
It provides the current status of the robot and enables its direct real-time control at 1 kHz with an external workstation PC (our desktop) connected via Ethernet.
We refer to the |frankaemika-fci-overview| for more information about its specifications.

.. |frankaemika-fci-overview| raw:: html

    <a href="https://frankaemika.github.io/docs/overview.html" target="_blank">Franka Emika FCI documentation</a>


Clone the franka_constrained_control project
------------------------------------------------

.. |GPL-3-license| raw:: html

   <a href=" https://www.gnu.org/licenses/gpl-3.0.nl.html" target="_blank">GPL-3 license</a>


.. note:: The GitHub repository is still private and will be made public in September 2022. 
          People from VUB and CU Boulder that are invited to the private repo, 
          may not share the content with others. Also not in publications as long as the repository is private. 

          :raw-html:`<font color="blue">  KELLY TO DO: </font>`

          *  :raw-html:`<font color="blue">  add </font>` 
             |GPL-3-license| 
             :raw-html:`<font color="blue">  to ALL files  </font>`

          *  :raw-html:`<font color="blue">  make the GitHub repo public  </font>`

          *  :raw-html:`<font color="blue">  change the link to the GitHub repo via the code block bash  </font>`


Clone the project and build ``libfranka`` and ``franka_ros``:

.. code-block:: bash

   git clone https://github.com/panda-brubotics/franka_constrained_control.git
   cd path/to/franka_constrained_control/libfranka
   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   cmake --build .
   cd ../../catkin_ws/src/franka_ros
   catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

.. warning::

   The path of ``-DFranka_DIR:PATH`` MUST be an absolute path to the libfranka's build directory.


Now that the project is built, you can use catkin_make without specifying the build type or the build directory, i.e.

.. code-block:: bash

   cd path/to/franka_constrained_control/catkin_ws
   catkin_make

Create your own project
------------------------

.. note :: If you want to make an extension or improvement to the franka_constrained_control project,
           it is better to clone the existing project as is explained in the previous section.
           However, if you like to make a new project with the same libfranka and ros version as we used,
           then you have to follow this section.

This tutorial is an adapted version of the official |frankaemika-linux-installation|.

.. |frankaemika-linux-installation| raw:: html

    <a href="https://frankaemika.github.io/docs/installation_linux.html" target="_blank">Franka Emika installation tutorial</a>

First of all make sure that ROS is installed :

.. code-block:: bash

   sudo apt install ros-melodic-libfranka ros-melodic-franka-ros

Before building from source, please uninstall existing installations of ``libfranka`` and ``franka_ros`` to avoid conflicts:

.. code-block:: bash

   sudo apt remove "*libfranka*"

Create a directory (or a git directory if you want to make a git repository) before installing ``libfranka`` and ``franka_ros``.

Install libfranka
^^^^^^^^^^^^^^^^^

To build libfranka, install the following dependencies from Ubuntu’s package manager:

.. code-block:: bash

   sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

Then, download the source code by cloning |frankaemika-libfranka-github| in your directory (or git repository):

.. |frankaemika-libfranka-github| raw:: html

    <a href="https://github.com/frankaemika/libfranka" target="_blank">libfranka</a>

.. code-block:: bash

   git clone --recursive https://github.com/frankaemika/libfranka
   cd libfranka

By default, this will check out the newest release of ``libfranka``.
However, we want to use the version *0.7.1*, so we have to change the branch:

.. code-block:: bash

   git checkout 0.7.1
   git submodule update

In the libfranka directory, create a build directory and run CMake:

.. code-block:: bash

   mkdir build
   cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   cmake --build .

Try to launch files in the ``build/examples`` directory to see if the installation is completed.

.. code-block:: bash

   cd build/examples
   ./echo_robot_state <fci-ip>
   ./print_joint_poses <fci-ip>

When you can run the examples, ``libfranka`` is installed properly, so you are ready to install ``franka_ros``.

Install franka_ros
^^^^^^^^^^^^^^^^^^

Go to your directory (or git repository) and create a catkin workspace :

.. code-block:: bash

   mkdir -p catkin_ws/src
   cd catkin_ws
   source /opt/ros/melodic/setup.sh
   catkin_init_workspace src

Then clone the |frankaemika-franka_ros-github| repository by executing the following command:

.. |frankaemika-franka_ros-github| raw:: html

    <a href="https://github.com/frankaemika/franka_ros" target="_blank">franka_ros</a>

.. code-block:: bash

   git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros

By default, this will check out the newest release of ``franka_ros``.
However, we want to use the version *0.6.0* since with the latest version there are some problems such that even the franka_example_controllers cannot be launched.
So we have to change the branch:

.. code-block:: bash

   cd src/franka_ros
   git checkout 0.6.0

Install any missing dependencies and build the packages:

.. code-block:: bash

   rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
   catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
   source devel/setup.sh

.. warning ::

   The path of ``-DFranka_DIR:PATH`` MUST be an absolute path to the libfranka's build directory.

.. note ::

   Once the problems with the latest franka_ros version are solved, we can try this latest franka_ros and accompanying libfranka version.

Since we don't want to work with git submodules, we remove all the submodules by following these steps: 

* Delete the relevant line from the .gitmodules file.

* Delete the relevant section from .git/config.

* Run git rm --cached path_to_submodule (no trailing slash).
