Vicon Motion Capture
=======================

.. role:: raw-html(raw)
    :format: html

Reading material
----------------

In :download:`this internship report <louis-lefevre-internship.pdf>` you can find a more detailed explanation of the Vicon motion capture room and Vicon Nexus. 
It explains how to define objects in Vicon Nexus for three different robots and for multiple scenarios. 

In :download:`this internship report <Harry_Report_2.pdf>` you can find some general info about the Vicon motion capture 
and also how to define objects in Vicon Nexus for some specific scenarios with the Panda robot. 

In :download:`this report <FrankaPanda_UseWithFCIandVicon.pdf>` the use of the Panda robot in the Vicon room is explained. 
It explains how to establish the ethernet connection and how to receive data from the Vicon motion capture system. 
It also explains how to make objects in Vicon Tracker (but currently we have Vicon Nexus as software in the lab). 

Possible problems
-----------------

Last year there were some problems with Vicon Nexus. 
If the cameras and the Lock Sync Box don't color blue when you start up Nexus, then it's highly probable that there is a firewall problem. 
How to solve this issue? 

    Go to Windows --> control panel --> all control panel items --> Windows Defender Firewall --> Allowed Apps (see attachment WindowsDefenderFirewall_ControlPanel) and check all Nexus applications (see attachment WindowsDefenderFirewall_AllowedApps)
    Go to Windows --> Windows Firewall --> Inbound rules --> enable and allow connection for all Nexus applications (see attachment WindowsFirewall_InboundRules)

You can also update the  Vicon Firmware via Vicon Firmware Update Utility (see attachment ViconFirmwareUpdateUtility)

Last year there was also a problem with the Vicon Datastream SDK 
When the vicon_bridge is giving occlusions when you clearly see the subject in Nexus, do the following checks.

    Go to C:\Program Files\Vicon\DataStream SDK\Win64\CPP and double click ViconDataStreamSDK_CPPTest
    When you get the same values in the Global and Local Translation/Rotation as in the Static Translation/Rotation (the standard non-overwritten values with zeros and ones), then you have to check that the Processing Output Level is "Kinematic Fit".
    Try the ViconDataStreamSDK_CPPTest again, when you now see the real values in the Global and Local Translation/Rotation, run the vicon_bridge
    You shouldn't see the "occlusion" warning again, which means you can call the subject position and orientation

Extra things that can be done if it still doesn't work: 

    install the newest version of the Datastream SDK (see attachment DatastreamSDK)
    install python (see attachment Python)
    run ViconDatastreanSDK_CPP test and check if the processing output level is kinematic fit (see attachment processing_output_level)