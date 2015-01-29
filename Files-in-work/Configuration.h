#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*******************************************************************************************************************************
  This configuration file contains the basic settings for your printer. Advanced settings can be found in Configuration_adv.h
********************************************************************************************************************************/

// User-specified version info of this build to display in [Pronterface, etc] terminal window during startup.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(Meek66e, Dolphin)" // Who made the changes.

//============================================================================================================================//
//=======================================================Quick Setup==========================================================//
//============================================================================================================================//

// en = English
// pl = Polish
// fr = French
// de = German
// es = Spanish
// ru = Russian
// it = Italian
// pt = Portuguese
// fi = Finnish
// an = Aragonese
// nl = Dutch
// ca = Catalan
// eu = Basque-Euskera

//Select your language from the list above
#define LANGUAGE_SELECTION en

// Define this to set a custom name for your generic printer
#define CUSTOM_MENDEL_NAME "3D Printer"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// 0 = Not defined/ manual input
// 1 = Printrbot Simple Maker 1405

// The following defines a predefined printer configuration stored in Printer_Lib.h
// If your printer is not listed, was home built, or modified please leave this as 0 and enter your values
// in the MANUAL SETTINGS section below
#ifndef PRINTER
  #define PRINTER 0
#endif

//============================================================================================================================//
//=======================================================Manual Settings======================================================//
//============================================================================================================================//
// These settings are for those who want to set up their printer manualy or have a modified printer

//======================================================Connection Setting====================================================//

// SERIAL_PORT selects which serial port should be used for communication with the host.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
#define BAUDRATE 250000

// This enables the serial port associated to the Bluetooth interface
//#define BTENABLED              

//=======================================================Printer Settings=====================================================//

// 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
// 11 = Gen7 v1.1, v1.2 = 11
// 12 = Gen7 v1.3
// 13 = Gen7 v1.4
// 2  = Cheaptronic v1.0
// 20 = Sethi 3D_1
// 3  = MEGA/RAMPS up to 1.2 = 3
// 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
// 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
// 35 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
// 4  = Duemilanove w/ ATMega328P pin assignment
// 5  = Gen6
// 51 = Gen6 deluxe
// 6  = Sanguinololu < 1.2
// 62 = Sanguinololu 1.2 and above
// 63 = Melzi
// 64 = STB V1.1
// 65 = Azteeg X1
// 66 = Melzi with ATmega1284 (MaKr3d version)
// 67 = Azteeg X3
// 7  = Ultimaker
// 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
// 77 = 3Drag Controller
// 8  = Teensylu
// 80 = Rumba
// 81 = Printrboard rev A-E (AT90USB1286)
// 82 = Brainwave (AT90USB646)
// 83 = SAV Mk-I (AT90USB1286)
// 84 = Printrboard rev F (AT90USB1286)
// 9  = Gen3+
// 70 = Megatronics
// 701= Megatronics v2.0
// 702= Minitronics v1.0
// 90 = Alpha OMCA board
// 91 = Final OMCA board
// 301 = Rambo
// 21 = Elefu Ra Board (v3)

//select which electronics board you have from the list above.
#ifndef MOTHERBOARD
  #define MOTHERBOARD 84
#endif

// This defines the number of extruders
#define EXTRUDERS 1

// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

// select which power supply you have from the list above.
#define POWER_SUPPLY 1


//=======================================================Thermal Settings====================================================//



//=======================================================Endstop Settings===================================================//



//===================================================Optional Equippment Settings===========================================//



//======================================================Auto Defining Settings==============================================//
// Note: These settings are auto defining and should not need to be changed unless problems occure 



//========================================================Safety Settings===================================================//
// Note: These settings prevent your printer from performing unsafe actions and should not be disabled under normal operation



//=========================================================Debug Settings==================================================//
// These settings are for debugging certain 3d printer settings and should not be activated for normal operation

// Define this to have the electronics keep the powersupply off on startup.
// #define PS_DEFAULT_OFF


