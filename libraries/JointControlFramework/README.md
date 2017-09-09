# RoveBoard! (for all places that aren't energia)

## Overview
Programmers: 
Primary structure: Drue Satterfield 
Individual files: Varied

This is rover's official Hardware Abstraction Layer (HAL), a series of functions designed to a) be able to be used on any microcontroller that rover uses (in theory, check which boards are supported further down) b) allow the programmer to work with the hardware on the microcontroller easily and accessibly with easy-to-read and understand function calls C) allow roveware files like RoveComm, which are designed to be used across rover, to be used on every microcontroller we're using by using the functions provided in this framework that are promised to work on every supported microcontroller. 

It's fairly similar to arduino or energia, if you know what those are (directly inspired by, in fact) but unlike those two, the functions in roveboard are well documented and flat out tell you WHAT parts of the hardware the functions use, without you having to know what they're specifically DOING to carry out the function. In this way, you know what's going on in the microcontroller when you use roveboard, while still getting the easy hardware-accessing universal functions to use. Also unlike the former two, each microcontroller not only has universally supported functions provided for roveware files to use on every microcontroller, but also a series of functions that take specific advantage of the board for main.cpp and related, non universal files to use if they wish. The latter functions will only work with their own processor, but that's perfectly fine for non-portable files like main.cpp.

The framework is designed to be used by any IDE and c++ compiler as well, so you can take it with you wherever you go...except for energia, which needs and has its own specific version.

## Files and folders
At the top, you'll see a series of files that follow the naming pattern of "RoveBoard.h" and "RoveBoard_(microcontroller name).h". The former is the primary include for roveware files like RoveComm to include, and returns a list of prototyped functions that are supported across the framework. But, Roveboard.h importantly doesn't return the actual definitions of those functions, just the names and the fact that they exist. The other roveboard files like "RoveBoard_TM4C1294NCPDT.h" return those functions and others that work specifically on the tm4c and define them. 

The folders inside RoveBoard are split into three categories; generics, which contain files that name the functions that every microcontroller in roveboard implements, utilities, which contain supporting functions, macros, typenames and classes that aren't device-specific so are there to be helpful to the entire framework, and lastly you'll see folders devoted to a specific processor like 'tm4c1294ncpdt'. Those folders contain the files that list and define the functions made for their processor, and what most users of RoveBoard will be interested in looking at.

## Supported microcontrollers
* Tiva tm4c1294ncpdt


## Dependencies
* The tiva tm4c1294ncpdt libraries depend on the Tivaware firmware, provided by texas instruments for using the tiva. You'll need to download it yourself and point the compiler to it so that the #Include's in all the tiva files can reference it.

## Usage
1) In your main.cpp, before anything else include the roveboard_x.h file for the processor you're using
2) From a coding standpoint, that's it. Make sure to read the H files for your processor, as they contain not only the functions you're free to use in your files, but also any warnings, bugs or cautions.
3) If needed, tell the compiler where the RoveBoard folder is
4) If needed, tell the linker where the .lib file is (should be in the processor folder, for whatever processor you're using)

99) If you're making a new roveware component, have it include "RoveBoard.h" to get all the generic, universally supported functions you can use.



## Adding more boards/functions to existing boards
1) Make a folder for it, and a RoveBoard_x.h for it. 
2) Make sure you define all the functions under generics before you put it in.
3) You'll need to generate a .lib file for your new library, so that linkers can actually use it. Most psuedo-professional IDE's will have ways for you to generate them.
4) Make sure it compiles and test it first; you can use the included example code from 2016's arm, if you want

