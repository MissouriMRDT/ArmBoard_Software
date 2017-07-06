# ArmBoardSoftware
Software Control for Robotic Arm

Welcome to the arm board software repo. As the name implies, this is where development on the arm board software happens. Very titular like that.

Repo overview: The repo is split into developmental branches as well as the master branch. The master branch is for development of the main arm program; peripheral software is developed in their own developmental branches.

--------------------------------------------------------------------------------------------------------------------------------

The arm board's primary task is simple in concept. The base station will tell the arm how it wants to move, and the arm board makes it 
happen. The devil pops up a little in the details; how the arm is to move is something that varies depending on how the user wants
the arm to be controlled. All in all, the arm board software can be thought of as being three modules working together. Communications,
control algorithm, and mechatronic handling. That is to say, the arm board gets messages (and sends some back), determines how to 
carry out that message, and then tells the motors/servos/moving parts to move.

Communications: The software manages messages from the base station, receiving commands and if needed sending back information. This is
done with the library called RoveComm, a communication library built for the Tiva 1294 microcontroller which implements ethernet.
The library, developed by the mars rover software architecture group, streamlines rover communications by simply passing along 
command ID's and command data.

Control algorithm: Once the board gets a command from base station, it needs to determine how to interpet the command's data and 
how to carry it out. This is entirely dependent on the control scheme being used for controlling the arm. For example, if the user
is simply telling a joint to go at a certain speed, then we interpret the command data as how fast to go. If we are carrying it out
with open loop control (that is, without any feedback from the moving parts at all, we're just telling it to go and assuming it does it)
then we simply tell the moving part to move at that speed. However, if we're using closed loop control (taking feedback from the moving
parts to make sure they go at the speed or whatever we've commanded them to go at) then we need to monitor their conditions and ensure
they're doing as we want. Similarly, if the program is set up for a positional scheme, then we interpret the command data as a position
that joint needs to go towards. The program is set up to handle both command schemes.

Mechatronic handling: The final step, once we've gotten a command and determined how we wish to carry it out, is to actually tell
the moving parts to, you know, move. How this is done depends on which motor we're moving, but typically it's just commanding H bridge circuits via gpio pins.

By the by, if any of these technical terms are confusing, there is some further information regarding several of these mentioned concepts
under RoveSoDrive -> Resources -> tutorials -> electrical
