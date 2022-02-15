#ifndef RoveCommManifest_h
#define RoveCommManifest_h
//RC:2.2

#include <stdint.h>
#include "RoveCommPacket.h"


//RoveComm Port IDs
#define RC_ROVECOMM_ETHERNET_UDP_PORT      	                 11000 
#define RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT       11001
#define RC_ROVECOMM_ETHERNET_BMSBOARD_PORT                   11002
#define RC_ROVECOMM_ETHERNET_POWERBOARD_PORT                 11003
#define RC_ROVECOMM_ETHERNET_CAMERABOARD_PORT                11004
#define RC_ROVECOMM_ETHERNET_N3_PORT                         11005
#define RC_ROVECOMM_ETHERNET_SHIMBLEBOARD_PORT               11006
#define RC_ROVECOMM_ETHERNET_ARMBOARD_PORT                   11007
#define RC_ROVECOMM_ETHERNET_SRA_ACTUATIONBOARD_PORT         11008
#define RC_ROVECOMM_ETHERNET_SRA_SENSORBOARD_PORT            11009
#define RC_ROVECOMM_ETHERNET_WIRELESS_ESTOP_PORT             11010
#define RC_ROVECOMM_ETHERNET_PR_CONTROLLER_PORT              11011
#define RC_ROVECOMM_ETHERNET_BSMS_PORT                       11012
#define RC_ROVECOMM_ETHERNET_BLACK_BOX_PORT                  11013
#define RC_ROVECOMM_ETHERNET_AUTONOMY_PORT                   11015

//IP Addresses
#define RC_ROVECOMM_SUBNET_IP_FIRST_OCTET     192
#define RC_ROVECOMM_SUBNET_IP_SECOND_OCTET    168
#define RC_ROVECOMM_SUBNET_IP_THIRD_OCTET       1
        
#define RC_ROCKETROVER900_FOURTHOCTET          82
#define RC_ROCKETBASE900_FOURTHOCTET           83
#define RC_ROCKETROVE58_FOURTHOCTET            84
#define RC_ROCKETBASE58_FOURTHOCTET            85
#define RC_ROCKETROVER24_FOURTHOCTET           86
#define RC_ROCKETROVERSPARE_FOURTHOCTET        87
        
#define RC_OPEN_FOURTHOCTET                    130
#define RC_DRIVEBOARD_FOURTHOCTET              131
#define RC_POWERBOARD_FOURTHOCTET              132
#define RC_BMSBOARD_FOURTHOCTET                133
#define RC_CAMERABOARD_FOURTHOCTET             134
#define RC_N3_FOURTHOCTET                      135
#define RC_SHIMBLEBOARD_FOURTHOCTET            136
#define RC_ARMBOARD_FOURTHOCTET                137
#define RC_SRAACTUATIONBOARD_FOURTHOCTET       138
#define RC_SRASENSORSBOARD_FOURTHOCTET         139
#define RC_WIRELESS_ESTOP_FOURTHOCTET          140
#define RC_PR_CONTROLLER_FOURTHOCTET           141
#define RC_BSMS_FOURTHOCTET                    142
#define RC_BLACKBOX_FOURTHOCTET                143
#define RC_AUTONOMY_FOURTHOCTET                144

#define RC_STREAMERPI_FOURTHOCTET              150
        
#define RC_GRANDSTREAM1_FOURTHOCTET            226
#define RC_GRANDSTREAM2_FOURTHOCTET            227

/*RoveCommDataIDs: (0, 65535)
 *AABCC
 *AA-Board Number
 *B -Command Type
 *CC-Command Number
 */ 
 
//Board numbers
#define _DRIVEBOARD_BOARDNUMBER      01*1000
#define _BMSBOARD_BOARDNUMBER        02*1000
#define _POWERBOARD_BOARDNUMBER      03*1000
#define _CAMERABOARD_BOARDNUMBER     04*1000    
#define _N3_BOARDNUMBER              05*1000    
#define _SHIMBLEBOARD_BOARDNUMBER    06*1000    
#define _ARMBOARD_BOARDNUMBER        7*1000    
#define _SRAACTUATION_BOARDNUMBER    8*1000    
#define _SRASENSORS_BOARDNUMBER      9*1000
#define _WIRELESS_ESTOP_BOARDNUMBER  10*1000
#define _PR_CONTROLLER_BOARDNUMBER   11*1000
#define _BSMS_BOARDNUMBER            12*1000
#define _BLACKBOX_BOARDNUMBER        13*1000
#define _LIGHTING_BOARDNUMBER        14*1000
#define _AUTONOMY_BOARDNUMBER        15*1000  

//Command Types
#define _TYPE_COMMAND     0*100
#define _TYPE_TELEMETRY   1*100
#define _TYPE_ERROR       2*100

//Telemetry Update Rate
#define ROVECOMM_UPDATE_RATE    100

///////////////////////////////////////////////////
//             RoveComm System Id's              //
///////////////////////////////////////////////////
#define RC_ROVECOMM_PING_DATA_ID                    1
#define RC_ROVECOMM_PING_REPLY_DATA_ID              2
#define RC_ROVECOMM_SUBSCRIBE_REQUEST_DATA_ID       3
#define	RC_ROVECOMM_UNSUBSCRIBE_REQUEST_DATA_ID     4
#define	ROVECOMM_INVALID_VERSION_DATA_ID            5
#define	ROVECOMM_NO_DATA_DATA_ID                    6


///////////////////////////////////////////////////
//                DriveBoard                     //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Drive Left Right
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID             00+_TYPE_COMMAND+_DRIVEBOARD_BOARDNUMBER
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DATATYPE           int16_t     // (-1000, 1000)-->(-100%, 100%)
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DATACOUNT          2           //[LeftSpeed, RightSpeed]
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DRIVEMAXFORWARD    1000
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DRIVEMAXREVERSE    -1000
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_DRIVEZERO          0
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_LEFTSPEEDENTRY     0
#define RC_DRIVEBOARD_DRIVELEFTRIGHT_RIGHTSPEEDENTRY    1

//Drive Left Right
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID            	01+_TYPE_COMMAND+_DRIVEBOARD_BOARDNUMBER
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATATYPE          	int16_t			// (-1000, 1000)-> (-100%, 100%)
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATACOUNT        	    6				//[LF, LM, LR, RF, M, RR]
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DRIVEMAXFORWARD   	1000
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DRIVEMAXREVERSE   	-1000
#define RC_DRIVEBOARD_DRIVEINDIVIDUAL_DRIVEZERO         	0

//TCP/////
//Watchdog Override
#define RC_DRIVEBOARD_WATCHDOG_DATAID           02+_TYPE_COMMAND+_DRIVEBOARD_BOARDNUMBER
#define RC_DRIVEBOARD_WATCHDOG_DATATYPE         uint8_t			
#define RC_DRIVEBOARD_WATCHDOG_DATACOUNT        1				
#define RC_DRIVEBOARD_WATCHDOG_TURN_OFF   	    1
#define RC_DRIVEBOARD_WATCHDOG_TURN_ON         	0

//Telemetry////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Drive Speed
#define RC_DRIVEBOARD_DRIVE_SPEED_DATAID                00+_TYPE_TELEMETRY+_DRIVEBOARD_BOARDNUMBER
#define RC_DRIVEBOARD_DRIVE_SPEED_DATATYPE			    int16_t //(-1000, 1000)-> (-100%, 100%)
#define RC_DRIVEBOARD_DRIVE_SPEED_DATACOUNT         	6      //[LF, LM, LR, RF, RM,RR]

//Error////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//WatchDog Triggered
#define RC_DRIVEBOARD_WACHDOGTRIGGERED_DATAID           00+_TYPE_ERROR+_DRIVEBOARD_BOARDNUMBER
#define RC_DRIVEBOARD_WACHDOGTRIGGERED_DATATYPE			uint8_t
#define RC_DRIVEBOARD_WACHDOGTRIGGERED_DATACOUNT       	1
#define RC_DRIVEBOARD_WACHDOGTRIGGERED_TRIGGERED		1
#define RC_DRIVEBOARD_WACHDOGTRIGGERED_NOTTRIGGERED		0

///////////////////////////////////////////////////
//                BMSBoard                       //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//BMS Suicide
#define RC_BMSBOARD_SUICIDE_DATAID            	00+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_SUICIDE_DATATYPE          	uint8_t		
#define RC_BMSBOARD_SUICIDE_DATACOUNT        	0		

//Software E-Stop
#define RC_BMSBOARD_SW_ESTOP_DATAID            	01+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_SW_ESTOP_DATATYPE          	uint8_t		
#define RC_BMSBOARD_SW_ESTOP_DATACOUNT        	1   //number of seconds until reboot, defaults to 1

//UDP/////
//Wireless E-Stop Enable
#define RC_BMSBOARD_ENABLE_WIRELESS_ESTOP_DATAID            02+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_ENABLE_WIRELESS_ESTOP_DATATYPE          uint8_t		
#define RC_BMSBOARD_ENABLE_WIRELESS_ESTOP_DATACOUNT        	1   //0 to disable wireless estop functionality, 1 to disable

//Wireless E-Stop 
#define RC_BMSBOARD_WIRELESS_ESTOP_DATAID            03+_TYPE_COMMAND+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_WIRELESS_ESTOP_DATATYPE          uint8_t		
#define RC_BMSBOARD_WIRELESS_ESTOP_DATACOUNT         1   //0 to keep rover alive, 1 to kill rover

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Main I_Meas
#define RC_BMSBOARD_MAINIMEASmA_DATAID          00+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_MAINIMEASmA_DATATYPE        float	//main current output
#define RC_BMSBOARD_MAINIMEASmA_DATACOUNT      	1

//Pack  V_Meas
#define RC_BMSBOARD_PACK_VMEAS_DATAID           01+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_PACK_VMEAS_DATATYPE         float	
#define RC_BMSBOARD_PACK_VMEAS_DATACOUNT        1

//Cell  V_Meas
#define RC_BMSBOARD_CELL_VMEAS_DATAID           02+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_CELL_VMEAS_DATATYPE         float	
#define RC_BMSBOARD_CELL_VMEAS_DATACOUNT        8

//Temp_Meas
#define RC_BMSBOARD_TEMPMEASmDEGC_DATAID        03+_TYPE_TELEMETRY+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_TEMPMEASmDEGC_DATATYPE      float	//Temperature Reading in Deg Celcius
#define RC_BMSBOARD_TEMPMEASmDEGC_DATACOUNT     1

//Error//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//Pack Overcurrent
#define RC_BMSBOARD_PACK_OVERCURRENT_DATAID		00+_TYPE_ERROR+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_PACK_OVERCURRENT_DATATYPE	uint8_t	
#define RC_BMSBOARD_PACK_OVERCURRENT_DATACOUNT	1	

//Cell Undervoltage
#define RC_BMSBOARD_CELL_UNDERVOLTAGE_DATAID		01+_TYPE_ERROR+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_CELL_UNDERVOLTAGE_DATATYPE		uint8_t	
#define RC_BMSBOARD_CELL_UNDERVOLTAGE_DATACOUNT		2 //bitmasked value

//Pack Undervoltage
#define RC_BMSBOARD_PACK_UNDERVOLTAGE_DATAID		02+_TYPE_ERROR+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_PACK_UNDERVOLTAGE_DATATYPE		uint8_t	
#define RC_BMSBOARD_PACK_UNDERVOLTAGE_DATACOUNT		1

//Pack Superhot
#define RC_BMSBOARD_PACK_SUPERHOT_DATAID		03+_TYPE_ERROR+_BMSBOARD_BOARDNUMBER
#define RC_BMSBOARD_PACK_SUPERHOT_DATATYPE	    uint8_t	
#define RC_BMSBOARD_PACK_SUPERHOT_DATACOUNT		1

///////////////////////////////////////////////////
//                PowerBoard                     //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//Motor bus enable
#define RC_POWERBOARD_MOTOR_BUSENABLE_DATAID        00+_TYPE_COMMAND+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_MOTOR_BUSENABLE_DATATYPE      uint8_t 		
#define RC_POWERBOARD_MOTOR_BUSENABLE_DATACOUNT     1   //[Enable/Disable][M1, M2, M3, M4, M5, M6]	(bitmask)
#define RC_POWERBOARD_MOTOR_BUSENABLE_ENABLE		    1
#define RC_POWERBOARD_MOTOR_BUSENABLE_DISABLE       0

//12V bus enable
#define RC_POWERBOARD_12V_BUSENABLE_DATAID          01+_TYPE_COMMAND+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_12V_BUSENABLE_DATATYPE        uint8_t 		
#define RC_POWERBOARD_12V_BUSENABLE_DATACOUNT       1   //[Enable/Disable][Act, Logic]  (bitmask)
#define RC_POWERBOARD_12V_BUSENABLE_ENABLE			    1
#define RC_POWERBOARD_12V_BUSENABLE_DISABLE         0

//30V bus enable
#define RC_POWERBOARD_30V_BUSENABLE_DATAID          02+_TYPE_COMMAND+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_30V_BUSENABLE_DATATYPE        uint8_t 		
#define RC_POWERBOARD_30V_BUSENABLE_DATACOUNT       1   //[Enable/Disable][12V Board, Rockets, Aux]  (bitmask)
#define RC_POWERBOARD_30V_BUSENABLE_ENABLE			    1
#define RC_POWERBOARD_30V_BUSENABLE_DISABLE         0

//Vacuum enable
#define RC_POWERBOARD_VACUUM_ENABLE_DATAID          03+_TYPE_COMMAND+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_VACUUM_ENABLE_DATATYPE        uint8_t 		
#define RC_POWERBOARD_VACUUM_ENABLE_DATACOUNT       1   //[Enable/Disable]
#define RC_POWERBOARD_VACUUM_ENABLE_ENABLE			    1
#define RC_POWERBOARD_VACUUM_ENABLE_DISABLE         0

//Patch Panel enable
#define RC_POWERBOARD_PATCH_PANELENABLE_DATAID      04+_TYPE_COMMAND+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_PATCH_PANELENABLE_DATATYPE    uint8_t 		
#define RC_POWERBOARD_PATCH_PANELENABLE_DATACOUNT   1   //[Enable/Disable][Panels 1-8]  (bitmask)
#define RC_POWERBOARD_PATCH_PANELENABLE_ENABLE			1
#define RC_POWERBOARD_PATCH_PANELENABLE_DISABLE     0

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Motor bus enabled
#define RC_POWERBOARD_MOTOR_BUSENABLED_DATAID        00+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_MOTOR_BUSENABLED_DATATYPE      uint8_t 		
#define RC_POWERBOARD_MOTOR_BUSENABLED_DATACOUNT     1   //[Enabled/Disabled][M1, M2, M3, M4, M5, M6]	(bitmask)

//12V bus enabled
#define RC_POWERBOARD_12V_BUSENABLED_DATAID          01+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_12V_BUSENABLED_DATATYPE        uint8_t 		
#define RC_POWERBOARD_12V_BUSENABLED_DATACOUNT       1   //[Enabled/Disabled][Act, Logic]  (bitmask)

//30V bus enabled
#define RC_POWERBOARD_30V_BUSENABLED_DATAID          02+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_30V_BUSENABLED_DATATYPE        uint8_t 		
#define RC_POWERBOARD_30V_BUSENABLED_DATACOUNT       1   //[Enabled/Disabled][12V Board, Rockets, Aux]  (bitmask)

//Vacuum enabled
#define RC_POWERBOARD_VACUUM_ENABLED_DATAID          03+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_VACUUM_ENABLED_DATATYPE        uint8_t 		
#define RC_POWERBOARD_VACUUM_ENABLED_DATACOUNT       1   //[Enabled/Disabled]

//Patch Panel enabled
#define RC_POWERBOARD_PATCH_PANELENABLED_DATAID      04+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_PATCH_PANELENABLED_DATATYPE    uint8_t 		
#define RC_POWERBOARD_PATCH_PANELENABLED_DATACOUNT   1   //[Enabled/Disabled][Panels 1-8]  (bitmask)

//Motor bus current
#define RC_POWERBOARD_MOTOR_BUS_CURRENT_DATAID       05+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_MOTOR_BUS_CURRENT_DATATYPE     float 		
#define RC_POWERBOARD_MOTOR_BUS_CURRENT_DATACOUNT    6   //[M1, M2, M3, M4, M5, M6]	

//12V bus current
#define RC_POWERBOARD_12V_BUS_CURRENT_DATAID         06+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_12V_BUS_CURRENT_DATATYPE       float 		
#define RC_POWERBOARD_12V_BUS_CURRENT_DATACOUNT      2   //[Act, Logic]  

//30V bus current
#define RC_POWERBOARD_30V_BUS_CURRENT_DATAID         07+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_30V_BUS_CURRENT_DATATYPE       float 		
#define RC_POWERBOARD_30V_BUS_CURRENT_DATACOUNT      3  //[12V Board, Rockets, Aux] 

//Vacuum current
#define RC_POWERBOARD_VACUUM_CURRENT_DATAID          8+_TYPE_TELEMETRY+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_VACUUM_CURRENT_DATATYPE        float 		
#define RC_POWERBOARD_VACUUM_CURRENT_DATACOUNT       1    

//Error//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//Motor bus overcurrent
#define RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATAID          00+_TYPE_ERROR+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATATYPE        uint8_t 		
#define RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATACOUNT       1   //[M1, M2, M3, M4, M5, M6, M7, Spare]	(bitmask)

//12V bus overcurrent
#define RC_POWERBOARD_12V_BUS_OVERCURRENT_DATAID          01+_TYPE_ERROR+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_12V_BUS_OVERCURRENT_DATATYPE        uint8_t 		
#define RC_POWERBOARD_12V_BUS_OVERCURRENT_DATACOUNT       1   //[Act, Logic]  (bitmask)

//30V bus overcurrent
#define RC_POWERBOARD_30V_BUS_OVERCURRENT_DATAID          02+_TYPE_ERROR+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_30V_BUS_OVERCURRENT_DATATYPE        uint8_t 		
#define RC_POWERBOARD_30V_BUS_OVERCURRENT_DATACOUNT       1  //[12V Board, Rockets, Aux]  (bitmask)

//Vacuum overcurrent
#define RC_POWERBOARD_VACUUM_OVERCURRENT_DATAID          03+_TYPE_ERROR+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_VACUUM_OVERCURRENT_DATATYPE        uint8_t 		
#define RC_POWERBOARD_VACUUM_OVERCURRENT_DATACOUNT       1   

//Patch Panel overcurrent
#define RC_POWERBOARD_PATCH_PANEL_OVERCURRENT_DATAID          04+_TYPE_ERROR+_POWERBOARD_BOARDNUMBER
#define RC_POWERBOARD_PATCH_PANEL_OVERCURRENT_DATATYPE        uint8_t 		
#define RC_POWERBOARD_PATCH_PANEL_OVERCURRENT_DATACOUNT       1   //[Panels 1-8] (bitmask)

///////////////////////////////////////////////////
//                CameraBoard                    //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////
//                NavigationBoard                //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////
//                ShimbleBoard                   //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Left Drive Gimbal Increment
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOINC_DATAID         00+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOINC_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOINC_DATACOUNT	   2		//[Pan, Tilt]

//Right Drive Gimbal Increment
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOINC_DATAID         01+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOINC_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOINC_DATACOUNT		2		//[Pan, Tilt]

//Left Main Gimbal Increment
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOINC_DATAID         02+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOINC_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOINC_DATACOUNT	  2		//[Pan, Tilt]

//Right Main Gimbal Increment
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOINC_DATAID         03+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOINC_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOINC_DATACOUNT	   2		//[Pan, Tilt]

//Left Drive Gimbal Absolute
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOABS_DATAID         04+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOABS_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_LEFT_DRIVE_SERVOABS_DATACOUNT	   2		//[Pan, Tilt]

//Right Drive Gimbal Absolute
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOABS_DATAID         05+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOABS_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_RIGHT_DRIVE_SERVOABS_DATACOUNT		2		//[Pan, Tilt]

//Left Main Gimbal Absolute
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOABS_DATAID         06+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOABS_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_LEFT_MAIN_SERVOABS_DATACOUNT	  2		//[Pan, Tilt]

//Right Main Gimbal Absolute
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOABS_DATAID         07+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOABS_DATATYPE       int16_t	//Degrees
#define RC_SHIMBLEBOARD_RIGHT_MAIN_SERVOABS_DATACOUNT	   2		//[Pan, Tilt]

//Initiate Startup Routine
#define RC_SHIMBLEBOARD_STARTUP_ROUTINE_DATAID         08+_TYPE_COMMAND+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_STARTUP_ROUTINE_DATATYPE       uint8_t	
#define RC_SHIMBLEBOARD_STARTUP_ROUTINE_DATACOUNT	   1        //1 - for running startup routine again

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Servo Position
#define RC_SHIMBLEBOARD_SERVODEG_DATAID       	00+_TYPE_TELEMETRY+_SHIMBLEBOARD_BOARDNUMBER
#define RC_SHIMBLEBOARD_SERVODEG_DATATYPE     	int16_t	//Degrees
#define RC_SHIMBLEBOARD_SERVODEG_DATACOUNT		8		//[S1][S2][S3][S4][S5][S6][S7][S8]

///////////////////////////////////////////////////
//                SRAActuationBoard              //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Z-Axis
#define RC_SRAACTUATION_Z_AXIS_DATAID     	00+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_Z_AXIS_DATATYPE   	int16_t
#define RC_SRAACTUATION_Z_AXIS_DATACOUNT  	1   //[-1000, 1000] speed

//Geneva Open Loop
#define RC_SRAACTUATION_GENEVA_OPENLOOP_DATAID     	01+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_GENEVA_OPENLOOP_DATATYPE   	int16_t
#define RC_SRAACTUATION_GENEVA_OPENLOOP_DATACOUNT  	1   //[-1000, 1000] speed

//TCP/////
//Chemicals
#define RC_SRAACTUATION_CHEMICALS_DATAID     	02+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_CHEMICALS_DATATYPE   	int16_t	
#define RC_SRAACTUATION_CHEMICALS_DATACOUNT  	3   //[Chemical 1, Chemical 2, Chemical 3]

//Geneva to position
#define RC_SRAACTUATION_GENEVA_TO_POS_DATAID     	03+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_GENEVA_TO_POS_DATATYPE   	uint8_t	
#define RC_SRAACTUATION_GENEVA_TO_POS_DATACOUNT  	1   //[Absolute position]

//Geneva increment position
#define RC_SRAACTUATION_GENEVA_INC_POS_DATAID     	04+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_GENEVA_INC_POS_DATATYPE   	int8_t	
#define RC_SRAACTUATION_GENEVA_INC_POS_DATACOUNT  	1   //[Relative position]

//Vacuum
#define RC_SRAACTUATION_VACUUM_DATAID     	05+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_VACUUM_DATATYPE   	uint8_t	
#define RC_SRAACTUATION_VACUUM_DATACOUNT  	1   //[0-off, 1-on]

//Limit switch override
#define RC_SRAACTUATION_LIMIT_SWITCH_OVERRIDE_DATAID     	06+_TYPE_COMMAND+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_LIMIT_SWITCH_OVERRIDE_DATATYPE   	uint8_t	
#define RC_SRAACTUATION_LIMIT_SWITCH_OVERRIDE_DATACOUNT  	1   //[Z-axis Top, Z-axis Bottom, Geneva Set, Geneva Home] (0-Turn off Limit Switch Override, 1-Turn on Limit Switch Override) (bitmasked)

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//UDP/////
//Geneva Current Position
#define RC_SRAACTUATION_GENEVA_CURRENT_POSITION_DATAID     	00+_TYPE_TELEMETRY+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_GENEVA_CURRENT_POSITION_DATATYPE   	uint8_t	
#define RC_SRAACTUATION_GENEVA_CURRENT_POSITION_DATACOUNT  	1		//[absolute position]	

//Limit Switch Triggered
#define RC_SRAACTUATION_LIMIT_SWITCH_TRIGGERED_DATAID     	00+_TYPE_TELEMETRY+_SRAACTUATION_BOARDNUMBER
#define RC_SRAACTUATION_LIMIT_SWITCH_TRIGGERED_DATATYPE   	uint8_t	
#define RC_SRAACTUATION_LIMIT_SWITCH_TRIGGERED_DATACOUNT  	1       //[Z-axis Top, Z-axis Bottom, Geneva Set, Geneva Home] (bitmasked)

///////////////////////////////////////////////////
//                SRASensorsBoard                //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//UV LED Enable
#define RC_SRASENSORSBOARD_UVLEDENABLE_DATAID     	00+_TYPE_COMMAND+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_UVLEDENABLE_DATATYPE   	uint8_t	
#define RC_SRASENSORSBOARD_UVLEDENABLE_DATACOUNT  	1		
#define RC_SRASENSORSBOARD_UVLEDENABLE_ENABLED		1
#define RC_SRASENSORSBOARD_UVLEDENABLE_DISABLED		0

//Capture Spectrometer Data
#define RC_SRASENSORSBOARD_CAPTURE_SPECTROMETER_DATA_DATAID     	01+_TYPE_COMMAND+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_CAPTURE_SPECTROMETER_DATA_DATATYPE   	uint16_t	
#define RC_SRASENSORSBOARD_CAPTURE_SPECTROMETER_DATA_DATACOUNT  	1   //number of readings

//Science Light Control
#define RC_SRASENSORSBOARD_LIGHT_CONTROL_DATAID     	02+_TYPE_COMMAND+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_LIGHT_CONTROL_DATATYPE   	uint8_t	
#define RC_SRASENSORSBOARD_LIGHT_CONTROL_DATACOUNT  	1		
#define RC_SRASENSORSBOARD_LIGHT_CONTROL_ENABLED		1
#define RC_SRASENSORSBOARD_LIGHT_CONTROL_DISABLED		0

//Capture MPPC Data
#define RC_SRASENSORSBOARD_CAPTURE_MPCC_DATA_DATAID     	03+_TYPE_COMMAND+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_CAPTURE_MPCC_DATA_DATATYPE   	uint16_t	
#define RC_SRASENSORSBOARD_CAPTURE_MPCC_DATA_DATACOUNT  	1   //number of readings

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Capture CO2 Data
#define RC_SRASENSORSBOARD_CO2_DATA_DATAID     04+_TYPE_TELEMETRY+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_CO2_DATA_DATATYPE   float	 
#define RC_SRASENSORSBOARD_CO2_DATA_DATACOUNT  1         //(ppm)

#define RC_SRASENSORSBOARD_O2_DATA_DATAID     05+_TYPE_TELEMETRY+_SRASENSORS_BOARDNUMBER
#define RC_SRASENSORSBOARD_O2_DATA_DATATYPE   float	 
#define RC_SRASENSORSBOARD_O2_DATA_DATACOUNT  4         //[partial pressure (mBar), temperature (C), concentration (ppm), barometric pressure (mBar)]

///////////////////////////////////////////////////
//                LightingBoard                  //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//TCP/////
//Headlight Intensity
#define RC_LIGHTINGBOARD_SETLEDINTENS_DATAID            	00+_TYPE_COMMAND+_LIGHTING_BOARDNUMBER
#define RC_LIGHTINGBOARD_SETLEDINTENS_DATATYPE          	uint8_t		//0-100%
#define RC_LIGHTINGBOARD_SETLEDINTENS_DATACOUNT        		1		

//LED RGB
#define RC_LIGHTINGBOARD_SETRGB_DATAID            			01+_TYPE_COMMAND+_LIGHTING_BOARDNUMBER
#define RC_LIGHTINGBOARD_SETRGB_DATATYPE          			uint8_t		//0-255 rgb value
#define RC_LIGHTINGBOARD_SETRGB_DATACOUNT        			3			//[red][green][blue]

//LED Function
#define RC_LIGHTINGBOARD_LEDCMND_DATAID            			02+_TYPE_COMMAND+_LIGHTING_BOARDNUMBER
#define RC_LIGHTINGBOARD_LEDCMND_DATATYPE          			uint8_t		//command number
#define RC_LIGHTINGBOARD_LEDCMND_DATACOUNT        			1			

//State Display
#define RC_LIGHTINGBOARD_STATE_DISPLAY_DATAID            			03+_TYPE_COMMAND+_LIGHTING_BOARDNUMBER
#define RC_LIGHTINGBOARD_STATE_DISPLAY_DATATYPE          			uint8_t //command number
#define RC_LIGHTINGBOARD_STATE_DISPLAY_DATACOUNT        			1   //(enum order: Solid Blue, Solid Red, Flashing Green)		

///////////////////////////////////////////////////
//                ArmBoard                       //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////
//UDP////
//MoveOpenLoop
#define RC_ARMBOARD_VELOCITY_DATAID         00+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_VELOCITY_DATATYPE       int16_t	//[-1000, 1000] %-1
#define RC_ARMBOARD_VELOCITY_DATACOUNT      6	//[J1][J2][J3][J4][J5][J6]
#define RC_ARMBOARD_VELOCITY_J1ENTRY        0
#define RC_ARMBOARD_VELOCITY_J2ENTRY        1
#define RC_ARMBOARD_VELOCITY_J3ENTRY        2
#define RC_ARMBOARD_VELOCITY_J4ENTRY        3
#define RC_ARMBOARD_VELOCITY_J5ENTRY        4
#define RC_ARMBOARD_VELOCITY_J6ENTRY        5

//Increment Position
#define RC_ARMBOARD_INCREMENTPOSITION_DATAID        02+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_INCREMENTPOSITION_DATATYPE      float	//[-1000, 1000] %-1
#define RC_ARMBOARD_INCREMENTPOSITION_DATACOUNT     6	//[J1][J2][J3][J4][J5][J6]
#define RC_ARMBOARD_INCREMENTPOSITION_J1ENTRY       0
#define RC_ARMBOARD_INCREMENTPOSITION_J2ENTRY       1
#define RC_ARMBOARD_INCREMENTPOSITION_J3ENTRY       2
#define RC_ARMBOARD_INCREMENTPOSITION_J4ENTRY       3
#define RC_ARMBOARD_INCREMENTPOSITION_J5ENTRY       4
#define RC_ARMBOARD_INCREMENTPOSITION_J6ENTRY       5

//IK Rover Increment
#define RC_ARMBOARD_IKINCROV_DATAID         04+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_IKINCROV_DATATYPE       float	
#define RC_ARMBOARD_IKINCROV_DATACOUNT      6	//[X][Y][Z][P][[Y][R]
#define RC_ARMBOARD_IKINCROV_XENTRY         0
#define RC_ARMBOARD_IKINCROV_YENTRY         1
#define RC_ARMBOARD_IKINCROV_ZENTRY         2
#define RC_ARMBOARD_IKINCROV_PITCH_ENTRY    3
#define RC_ARMBOARD_IKINCROV_YAW_ENTRY      4
#define RC_ARMBOARD_IKINCROV_ROLL_ENTRY     5

//IK Wrist Increment
#define RC_ARMBOARD_IKINCWRIST_DATAID       05+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_IKINCWRIST_DATATYPE     float	
#define RC_ARMBOARD_IKINCWRIST_DATACOUNT    6	//[X][Y][Z][P][[Y][R]
#define RC_ARMBOARD_IKINCWRIST_XENTRY       0
#define RC_ARMBOARD_IKINCWRIST_YENTRY       1
#define RC_ARMBOARD_IKINCWRIST_ZENTRY       2
#define RC_ARMBOARD_IKINCWRIST_PITCH_ENTRY  3
#define RC_ARMBOARD_IKINCWRIST_YAW_ENTRY    4
#define RC_ARMBOARD_IKINCWRIST_ROLL_ENTRY   5


//Gripper Open Loop
#define RC_ARMBOARD_GRIPPER_DATAID     		10+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_GRIPPER_DATATYPE   		int16_t	//[-1000, 1000] %-1
#define RC_ARMBOARD_GRIPPER_DATACOUNT  		1

//TCP////
//MoveToAngle
#define RC_ARMBOARD_MOVETOPOSITION_DATAID      01+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_MOVETOPOSITION_DATATYPE    float	//deg-1
#define RC_ARMBOARD_MOVETOPOSITION_DATACOUNT   6	//[J1][J2][J3][J4][J5][J6]
#define RC_ARMBOARD_MOVETOPOSITION_J1ENTRY     0
#define RC_ARMBOARD_MOVETOPOSITION_J2ENTRY     1
#define RC_ARMBOARD_MOVETOPOSITION_J3ENTRY     2
#define RC_ARMBOARD_MOVETOPOSITION_J4ENTRY     3
#define RC_ARMBOARD_MOVETOPOSITION_J5ENTRY     4
#define RC_ARMBOARD_MOVETOPOSITION_J6ENTRY     5

//IK Absolute POS
#define RC_ARMBOARD_IKABSPOS_DATAID         03+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_IKABSPOS_DATATYPE       float	
#define RC_ARMBOARD_IKABSPOS_DATACOUNT      6	//[X][Y][Z][P][[Y][R]
#define RC_ARMBOARD_IKABSPOS_XENTRY         0
#define RC_ARMBOARD_IKABSPOS_YENTRY         1
#define RC_ARMBOARD_IKABSPOS_ZENTRY         2
#define RC_ARMBOARD_IKABSPOS_PITCH_ENTRY    3
#define RC_ARMBOARD_IKABSPOS_YAW_ENTRY      4
#define RC_ARMBOARD_IKABSPOS_ROLL_ENTRY     5

//Set Closed Loop Control
#define RC_ARMBOARD_SET_CLOSED_LOOP_DATAID     	06+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_SET_CLOSED_LOOP_DATATYPE   	uint8_t	//[0=disable, 1=enable]
#define RC_ARMBOARD_SET_CLOSED_LOOP_DATACOUNT  	1	
#define RC_ARMBOARD_SET_CLOSED_LOOP_DISABLE		0
#define RC_ARMBOARD_SET_CLOSED_LOOP_ENABLE		1

//Laser Control
#define RC_ARMBOARD_LASER_CONTROL_DATAID            11+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_LASER_CONTROL_DATATYPE          uint8_t	//[0=disable, 1=enable]
#define RC_ARMBOARD_LASER_CONTROL_DATACOUNT         1

//Solenoid Control
#define RC_ARMBOARD_SOLENOID_DATAID             12+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_SOLENOID_DATATYPE           uint8_t	//[0=disable, 1=enable]
#define RC_ARMBOARD_SOLENOID_DATACOUNT          1

//Watchdog Override
#define RC_ARMBOARD_WATCHDOG_OVERRIDE_DATAID        13+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_WATCHDOG_OVERRIDE_DATATYPE      uint8_t	//[0=disable, 1=enable]
#define RC_ARMBOARD_WATCHDOG_OVERRIDE_DATACOUNT     1

//Limit Switch Override
#define RC_ARMBOARD_LIMIT_SWITCH_DATAID             14+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_LIMIT_SWITCH_DATATYPE           uint8_t	//[0=disable, 1=enable]
#define RC_ARMBOARD_LIMIT_SWITCH_DATACOUNT          1

//ODrive Reboot
#define RC_ARMBOARD_REBOOT_ODRIVE_DATAID             15+_TYPE_COMMAND+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_REBOOT_ODRIVE_DATATYPE           uint8_t	//[1=reboot]
#define RC_ARMBOARD_REBOOT_ODRIVE_DATACOUNT          1

//Telemetry//////////////////////////////////////////////////////////////////////////////////////////////
//UDP////
//Motor Currents
#define RC_ARMBOARD_MOTORCURRENTS_DATAID     	00+_TYPE_TELEMETRY+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_MOTORCURRENTS_DATATYPE   	float	//A
#define RC_ARMBOARD_MOTORCURRENTS_DATACOUNT  	7       //[M1][M2][M3][M4][M5][M6][M7]
#define	RC_ARMBOARD_MOTORCURRENTS_M1ENTRY		0
#define	RC_ARMBOARD_MOTORCURRENTS_M2ENTRY		1
#define	RC_ARMBOARD_MOTORCURRENTS_M3ENTRY		2
#define	RC_ARMBOARD_MOTORCURRENTS_M4ENTRY		3
#define	RC_ARMBOARD_MOTORCURRENTS_M5ENTRY		4
#define	RC_ARMBOARD_MOTORCURRENTS_M6ENTRY		5
#define	RC_ARMBOARD_MOTORCURRENTS_M7ENTRY		6

//Joint Angles
#define RC_ARMBOARD_JOINTANGLES_DATAID     	01+_TYPE_TELEMETRY+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_JOINTANGLES_DATATYPE   	float    //deg
#define RC_ARMBOARD_JOINTANGLES_DATACOUNT  	6		 //[J1][J2][J3][J4][J5][J6]
#define	RC_ARMBOARD_JOINTANGLES_J1ENTRY		0
#define	RC_ARMBOARD_JOINTANGLES_J2ENTRY		1
#define	RC_ARMBOARD_JOINTANGLES_J3ENTRY		2
#define	RC_ARMBOARD_JOINTANGLES_J4ENTRY		3
#define	RC_ARMBOARD_JOINTANGLES_J5ENTRY		4
#define	RC_ARMBOARD_JOINTANGLES_J6ENTRY		5

//Motor Velocities
#define RC_ARMBOARD_MOTORVELOCITIES_DATAID     	02+_TYPE_TELEMETRY+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_MOTORVELOCITIES_DATATYPE   	float    //deg
#define RC_ARMBOARD_MOTORVELOCITIES_DATACOUNT  	6		 //[M1][M2][M3][M4][M5][M6]
#define	RC_ARMBOARD_MOTORVELOCITIES_M1ENTRY		0
#define	RC_ARMBOARD_MOTORVELOCITIES_M2ENTRY		1
#define	RC_ARMBOARD_MOTORVELOCITIES_M3ENTRY		2
#define	RC_ARMBOARD_MOTORVELOCITIES_M4ENTRY		3
#define	RC_ARMBOARD_MOTORVELOCITIES_M5ENTRY		4
#define	RC_ARMBOARD_MOTORVELOCITIES_M6ENTRY     5

//IK Coordinates
#define RC_ARMBOARD_IKCOORDINATES_DATAID        03+_TYPE_TELEMETRY+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_IKCOORDINATES_DATATYPE      float    //deg
#define RC_ARMBOARD_IKCOORDINATES_DATACOUNT     6		 //[J1][J2][J3][J4][J5][J6]
#define	RC_ARMBOARD_IKCOORDINATES_J1ENTRY       0
#define	RC_ARMBOARD_IKCOORDINATES_J2ENTRY       1
#define	RC_ARMBOARD_IKCOORDINATES_J3ENTRY       2
#define	RC_ARMBOARD_IKCOORDINATES_J4ENTRY       3
#define	RC_ARMBOARD_IKCOORDINATES_J5ENTRY       4
#define	RC_ARMBOARD_IKCOORDINATES_J6ENTRY       5

//LS Values
#define RC_ARMBOARD_LSVALUES_DATAID        04+_TYPE_TELEMETRY+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_LSVALUES_DATATYPE      uint8_t    //
#define RC_ARMBOARD_LSVALUES_DATACOUNT     1		 //[Base Tilt Up, Base Tilt Down, Elbow Tilt Up, Elbow Tilt Down]

//Watchdog Status
#define RC_ARMBOARD_WATCHDOGSTATUS_DATAID        05+_TYPE_ERROR+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_WATCHDOGSTATUS_DATATYPE      uint8_t 
#define RC_ARMBOARD_WATCHDOGSTATUS_DATACOUNT     1    //0-WD Triggered, 1-WD Triggered

//Error//////////////////////////////////////////////////////////////////////////////////////////////
//TCP////
#define RC_ARMBOARD_ENCODERSTATUS_DATAID        00+_TYPE_ERROR+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_ENCODERSTATUS_DATATYPE      uint8_t //[E1][E2][E3][E4][E5][E6]
#define RC_ARMBOARD_ENCODERSTATUS_DATACOUNT     1    //0-Encoder Fine, 1-Encoder Failure

#define RC_ARMBOARD_ODRIVEERROR_DATAID        01+_TYPE_ERROR+_ARMBOARD_BOARDNUMBER
#define RC_ARMBOARD_ODRIVEERROR_DATATYPE      uint8_t 
#define RC_ARMBOARD_ODRIVEERROR_DATACOUNT     3    //[Base Left, Base Right, ELbow Left, Elbow Right, Wrist Left, Wrist Right][Error Type][Error]

///////////////////////////////////////////////////
//                 AutonomyBoard                 //
///////////////////////////////////////////////////
//Commands//////////////////////////////////////////////////////////////////////////////////////////////

//Telemetry////////////////////////////////////////////////////////////////////////////////////////////

#endif // RoveCommManifest_h