/*   Programmer: Drue Satterfield, David Strickland
 *
 *   This is the external include for the Joint Interface Framework.
 *   Framework description here: https://github.com/MST-MRDT/ArmBoardSoftware/blob/development/libraries/ControlFramework/Framework%20Overview.docx
 *   Used for controlling output devices and easily manipulating joints. The user will only ever interact with the
 *   Joint Interface, which keeps track of an output device which moves that joint, and an algorithm for controlling that joint.
 *
 *   Files: The AbstractFramework files are the top level abstract classes that form the backbone of the framework. Firmware is sorted in the firmware include
 *   file. Enums, constants, and macros are kept in FrameworkUtilities file. The rest of the files are derivatives of the abstract classes; include the ones
 *   your program specifically uses.
 *
 *   necessary libraries outside of the framework:
 *   This program is meant to use energia libraries as well as the RoveDynamixel library and the PwmReader library and the PwmWriter library
 *
 *   implemenation notes:
 *   The user is supposed to use the library in this fashion:
 *   0) Include the derivative classes you want to use after including this file
 *   1) construct the output device class/classes representing the output device/devices (a motor controller, an h bridge, etc) used to move the joint
 *   2) if the joint is closed loop controlled, then the user must also construct a feedback device class representing the feedback device equipped on the joint as well as
 *      construct the IOAlgorithm class representing the closed loop algorithm they wish to use to control the joint
 *   3) Finally, construct the JointInterface class that represents this joint by passing it in the output device/devices and the constructed feedback device and closed loop algorithm, if used
 *      On top of those, the constructor also will have the user specify what kind of values they will pass the joint interface class, such as positional values or speed values. This is so
 *      that the interface knows how to properly interpret commands
 *   4) From here on out, the user should be able to simply call on the joint interface class for that joint to control the joint.
 *   note) When I say joint interface class/IOAlgorithm/output device/feedback device, I mean their specific derived classes representing the specific thing used on the joint, the formers are
 *         all abstract superclasses.
 *
 *   In order to implement modules into the program:
 *   Inherit from the proper abstract class.
 *   If you are implementing an algorithm, and that algorithm doens't use feedback, then
 *   go down to Joint Interface's selector method and put in the logic to select your new algorithm accordingly.
 *   Don't forget to make a constructor; if it's a device class then it should take in whatever parameters it needs to output properly such as hardware pins being used.
 *   If it's an open loop algorithm class, then the constructor likely doesn't take anything and all it needs to do internally is set the algorithm's input and output types.
 *   Also for open loop algorithm classes, their class setup must specify that JointInterface is a friend class (as the constructor should be protected since the user never needs it)
 *   If it's a closed loop algorithm class, then the constructor should take in any parameters used to configure the algorithm as well as internally set the input and output types.
 *   For closed loop algorithm classes, the constructor should be public since the user will need to construct them personally
 *
 *   Once you do implement a new module, make sure to record it in the modules in the 'modules in the framework' text file that 
 *   should be in the framework libarary's directory
*/

#ifndef JOINTCONTROLFRAMEWORK_H_
#define JOINTCONTROLFRAMEWORK_H_

#include "JointFrameworkUtilities.h"
#include "AbstractFramework.h"

#include "TiltJoint.h"
#include "SingleMotorJoint.h"
#include "RotateJoint.h"

#endif
