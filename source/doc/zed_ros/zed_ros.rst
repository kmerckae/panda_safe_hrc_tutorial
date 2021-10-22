ZED 2 with ROS Integration
===========================

.. role:: raw-html(raw)
    :format: html

Getting started with ROS and ZED
---------------------------------------------

.. |Stereolabs_ZED2_ROS_integration| raw:: html

   <a href="https://www.stereolabs.com/docs/ros/" target="_blank">getting started with ROS and ZED </a>

.. |Nvidia_JetPack_ubuntu| raw:: html

   <a href="https://developer.nvidia.com/embedded/jetpack" target="_blank">JetPack 4.6</a>

.. |Nvidia_Jetson_Xavier_NX_upgrade_Ubuntu20| raw:: html

   <a href="https://carlosedp.medium.com/upgrading-your-nvidia-jetson-xavier-nx-to-latest-ubuntu-focal-fossa-20-04-5e92ccc5a66" target="_blank">this article</a>

.. |ROS_Melodic_installation| raw:: html

   <a href="http://wiki.ros.org/melodic/Installation/Ubuntu" target="_blank">ROS Melodic</a>

.. |Stereolabs_github_zed_ros_wrapper| raw:: html

   <a href="https://github.com/stereolabs/zed-ros-wrapper" target="_blank">zed-ros-wrapper</a>

.. |Stereolabs_github_zed_ros_interfaces| raw:: html

   <a href="https://github.com/stereolabs/zed-ros-interfaces" target="_blank">zed-ros-interfaces</a>

.. |Stereolabs_github_zed_ros_examples| raw:: html

   <a href="https://github.com/stereolabs/zed-ros-examples" target="_blank">zed-ros-examples</a>

.. |Stereolabs_ZED_ROS_integration_buildpackages| raw:: html

   <a href="https://www.stereolabs.com/docs/ros/#build-the-packages " target="_blank">build the packages</a>


Read and follow the |Stereolabs_ZED2_ROS_integration| tutorial.

Below you will find some extra information on top of what the tutorial explains for the installation on the NVIDIA Jetson Xavier NX. 

*  We have installed |Nvidia_JetPack_ubuntu| on our NVIDIA Jetson Xavier NX and because this filesystem is based on Ubuntu 18.04, 
   we will follow the ROS installation procedure for Ubuntu 18.04, as such we have to install ROS Melodic. 
   If you want to upgrade the NVIDIA Jetson Xavier NX to Ubuntu 20.04, you can read |Nvidia_Jetson_Xavier_NX_upgrade_Ubuntu20|. 
   We didn't do this upgrade and worked on Ubuntu 18.04. 
*  For the installation of the ZED SDK for JetPack 4.6, we refer to :ref:`Install the ZED SDK on NVIDIA Jetson Xavier NX <Install_ZED_SDK_JetsonXavierNX>`. 
*  Follow the ROS tutorial to install |ROS_Melodic_installation|. 
   We have installed the *Desktop Install* version instead of the *Desktop-Full Install* version. 

   .. note::
       “ROS Desktop Full” is a more complete package, however it is not recommended for embedded platforms; 
       2D/3D simulators will be installed, requiring increased storage space and compute power.

*  To build the ZED ROS packages (|Stereolabs_github_zed_ros_wrapper|, |Stereolabs_github_zed_ros_interfaces|, and |Stereolabs_github_zed_ros_examples|)
   we have followed the |Stereolabs_ZED_ROS_integration_buildpackages| section. 

   .. note::
       If you install the |Stereolabs_github_zed_ros_wrapper| package, 
       then the |Stereolabs_github_zed_ros_interfaces| package is already included as a submodule. 
       As such, you will get an error *Rosdep experienced an error: Multiple packages found with the same name "zed_interfaces"*:
       zed-ros-interfaces and zed-ros-wrapper/zed-ros-interfaces.
       So on the Jetson Xavier NX you only have to install the |Stereolabs_github_zed_ros_wrapper| and |Stereolabs_github_zed_ros_examples| packages. 

*  








