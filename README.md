# ArmBoardSoftware
Software Control for Robotic Arm

Welcome to the arm board software repo. As the name implies, this is where development on the arm board software happens. Very titular like that.

Repo overview: The repo is split into developmental branches for in progress programs, and the master branch is reserved for code that is
actually working in some form or another. If it's blank, it means everything is in development.

The arm board's primary task is simple in concept. The base station will tell the arm how it wants to move, and the arm board makes it 
happen. The devil pops up a little in the details; how the arm is to move is something that varies depending on how the user wants
the arm to be controlled. All in all, the arm board software can be thought of as being three modules working together. Communications,
control algorithm, and mechatronic handling. That is to say, the arm board gets messages (and sends some back), determines how to 
carry out that message, and then tells the motors/servos/moving parts to move.

Communications: The software manages messages from the base station, receiving commands and if needed sending back information. This is
done with the library called RoveComm, a communication library built for the Tiva 1294 microcontroller which implements ethernet.
The library, developed by the mars rover software architecture group, streamlines rover communications by simply passing along 
command ID's and command data. The program also must handle communications with the rover endefector (hand), as the endef board's
communications pass through the arm board back to base station and vice versa. Unlike the commands meant for the arm, the endef 
data packets just get passed through to their intended recipient without any further input from the arm's part. 

Control algorithm: Once the board gets a command from base station, it needs to determine how to interpet the command's data and 
how to carry it out. This is entirely dependent on the control scheme being used for controlling the arm. For example, if the user
is simply telling a joint to go at a certain speed, then we interpret the command data as how fast to go. If we are carrying it out
with open loop control (that is, without any feedback from the moving parts at all, we're just telling it to go and assuming it does it)
then we simply tell the moving part to move at that speed. However, if we're using closed loop control (taking feedback from the moving
parts to make sure they go at the speed or whatever we've commanded them to go at) then we need to monitor their conditions and ensure
they're doing as we want. Similarly, if the program is set up for a positional scheme, then we interpret the command data as a position
that joint needs to go towards.

Mechatronic handling: The final step, once we've gotten a command and determined how we wish to carry it out, is to actually tell
the moving parts to, you know, move. How this is done is entirely dependent on what the moving parts actually are. A brushed DC motor
we might send a PWM signal. A brushed DC motor with a motor controller we might control in some other way. A servo we need to send 
a certain PWM pulse, a dynamixel brand servo we need to send a certain serial packet, etc etc. The program needs to be set up 
so that each joint gets commanded properly. 

By the by, if any of these technical terms are confusing, there is some further information regarding several of these mentioned concepts
under RoveSoDrive -> Resources -> tutorials -> electrical
