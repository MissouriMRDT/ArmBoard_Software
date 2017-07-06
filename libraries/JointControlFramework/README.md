# Joint Control Framework

## Overview
Programmers: Drue Satterfield, David Strickland

Used for controlling output devices and easily manipulating joints. The user will only ever interact with the Joint Interface, which keeps track of an output device which moves that joint, and an algorithm for controlling that joint.

[More Details Here](https://github.com/MST-MRDT/ArmBoardSoftware/wiki/Joint-control-framework-overview)

## Files
The `AbstractFramework.h` and `AbstractFramework.cpp` files holds the top level abstract classes that form the backbone of the framework. Enums, constants, and macros are kept in `JointFrameworkUtilities.h` file. `JointControlFramework.h` is the primary external include for the framework. The rest of the files define subclasses of the abstract base classes. 

## Dependencies
* [Energia Libraries](https://github.com/energia/Energia)
* [RoveDynamixel](https://github.com/MST-MRDT/RoveWare)
* PwmReader
* PwmWriter

## Usage
0) Include `JointControlFramework.h`. Then, include the derivative classes you want to use.
1) Construct the `OutputDevice` representing the device (motor controller, h-bridge, etc) used to move the joint.
2) If the joint is closed-loop controlled, then also construct a `FeedbackDevice` representing the device on the joint and the `IOAlgorithm` representing the desired closed-loop algorithm.
3) Finally, construct the `JointInterface` that represents this joint by passing in the `OutputDevice`, the `FeedbackDevice`, and the `IOAlgorithm`. The constructor will also require what kind of values the `JointInterface` should expect, such as positional values or speed values. This is so that the interface knows how to properly interpret commands.
4) The user should now be able to control the joint using the `JointInterface` object.

## Modules
### Joint Interfaces
Interface for controlling the overall joint from the main program's perspective. It handles all duties of controlling the joint.
* `SingleMotorJoint` Joint controlled by a singular motor device
* `RotateJoint` Differential joint rotational motion. Two motors move in opposite directions to control the joint.
* `TiltJoint` Differential joint tilt motion. Two motors move in the same direction to control the joint.

### Algorithms
Algorithms convert the input from base station to whatever is needed to interpret the command.
* `Spd to Spd no feedback` For inputting speed and outputting speed without feedback. Open loop speed control.
* `PIAlgorithm` Closed loop algorithm, using PI logic. Logic is generalized, PI constants are accepted through constructors. To be used when position is recieved from the base station and the speed is to be sent to the device, which in turn, returns feedback of the device's current location. Note that this algorithm needs to be looped externally until `JointInterface` returns am `OutputComplete` status, as each call only executes the PI loop once instead of waiting until completion.

### Output Devices
Controls the devices which move the arm, such as motor controllers, using the hardware specifics of the devices, such as what GPIO pins.
* `DirectDiscreteHBridge` An h-bridge made out of discrete components (not an IC), and the microcontroller lines directly control it (no other devices in between them). Two inputs, NTransistor1 and NTransistor2. It's assumed the other two transistors will be p-type transistors so they don't need control lines to the microcontroller. 
* `Sdc2130` A brushed DC motor controller, capable of being controlled via PWM or serial, and controlling the motor based on speed, position, or torque with feedback capabilities. Currently the only implemented modes are controlling it via PWM and moving by taking in a speed position. (Best we got it working is to make it slightly increment position when we tell it to move. So the only set up movement is for the device to take in a speed value, look at the value and if it's greater than 0, slightly move it up, and if it's less than 0 slightly move it down.)
* `DynamixelController` Interfaces with either MX or AX dynamixel. They can operate in Wheel, Joint, and Multi-rotation modes which take speed, position, and position respectively. Requires `RoveDynamixel.h`.
* `GenPwmPHaseHBridge` A class that represents any case where the h-bridge is controlled via two pins, specifically a speed pin controlled by PWM and a direction/phase pin.
* `RCContinuousServo` Generic class for any RC Continuous Servo device.
* `VNH5019` VNH5019 H-bridge IC

### Feedback Devices
Feedback devices are used to help determine where the arm is and what steps need to be taken. Used by the `IOAlgorithm` class to perform looping.
* `Ma3Encoder12b` MA3 magnetic encoder, 12 bit pwm version. Communicates via PWM, 12-bit resolution of degrees over 360 degrees.

## Extension
In order to implement new modules into the framework:
1) Inherit from the proper abstract class.
2) Define a constructor.
   * If it's a device class then it should take in whatever parameters it needs to output properly such as hardware pins.
   * If it's an open-loop algorithm class, then the constructor likely doesn't take anything and all it needs to do internally is set the algorithm's input and output types. Also for open-loop algorithm classes, their class setup must specify that `JointInterface` is a friend class (as the constructor should be protected since the user never needs it).
   * If it's a closed-loop algorithm class, then the constructor should take in any parameters used to configure the algorithm as well as internally set the input and output types. For closed-loop algorithm classes, the constructor should be public since the user will need to construct them personally.
3) Document the module in this README.