# RoveJoint #

Joint class for the Missouri S&T IoT Rover, Networked Tiva C microcontroller boosterpacks based on Energia IDE platform for the University Rover Challenge.

This library has both a Brushed DC Motor implementation and a currently WIP Brushless Motor implementation which utilizies a ODrive interfacing library to send commands to the ODrive brushless motor controller using their ASCII protocol. 

The development of this library is heading towards being able to use these classes interchangeably depending on whether a joint utilizes brushed or brushless motors so that all of the differences are abstracted away from the end user.

<pre>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Depends on RoveHalTiva, RoveMotor, RoveEncoder please install from:
// https://github.com/MissouriMRDT/RoveHalTiva 
// https://github.com/MissouriMRDT/RoveMotor
// https://github.com/MissouriMRDT/RoveEncoder
// https://github.com/MissouriMRDT/RovesODrive_Software
</pre>