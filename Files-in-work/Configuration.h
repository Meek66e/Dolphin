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
//#define CUSTOM_MENDEL_NAME "3D Printer"

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

// Uncomment the following line to enable CoreXY kinematics
// #define COREXY

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

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

// select which power supply you have from the list above.
#define POWER_SUPPLY 1


//=======================================================Thermal Settings====================================================//

// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 60 is 100k Maker's Tool Works Kapton Bed Thermister
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)

//Select your thermistor from the list above for each of your sensors
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 0

// This makes temp sensor 1 a redundant sensor for sensor 0. 
// If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
//#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define BED_MAXTEMP 150

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX 255 // limits current to nozzle while PID is active; 255=full current

// Enter your PID values for your hot end. If you don't know your values refer to WIKI for instructions to calibrate this later
#ifdef PIDTEMP
  #define  DEFAULT_Kp 22.2
  #define  DEFAULT_Ki 1.08
  #define  DEFAULT_Kd 114
#endif

// Uncomment the following line to enable bed PID and disable bang-bang.
//#define PIDTEMPBED

//If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis.
//#define BED_LIMIT_SWITCHING

#define MAX_BED_POWER 255 // limits current to bed; 255=full current

// Enter your PID values for your bed. If you don't know your values refer to WIKI for instructions to calibrate this later
#ifdef PIDTEMPBED
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4
#endif

//=======================================================Endstop Settings===================================================//

// Comment this out for optical endstops. Leave active for mechanical endstops. If you have a combination refer to WIKI
#define ENDSTOPPULLUPS

// set to true to invert the logic of the endstop.
const bool X_MIN_ENDSTOP_INVERTING = false; 
const bool Y_MIN_ENDSTOP_INVERTING = false; 
const bool Z_MIN_ENDSTOP_INVERTING = true; 
const bool X_MAX_ENDSTOP_INVERTING = false; 
const bool Y_MAX_ENDSTOP_INVERTING = false; 
const bool Z_MAX_ENDSTOP_INVERTING = false; 
//#define DISABLE_MAX_ENDSTOPS
//#define DISABLE_MIN_ENDSTOPS

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z true
#define DISABLE_E false // For all extruders

// Invert movment direction
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false

// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
#define X_MAX_POS_DEFAULT 100
#define X_MIN_POS_DEFAULT 0
#define Y_MAX_POS_DEFAULT 100
#define Y_MIN_POS_DEFAULT 0
#define Z_MAX_POS_DEFAULT 115
#define Z_MIN_POS_DEFAULT 0

// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 0

//=====================================================Stepper Settings=====================================================//

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.0,80.0,2020,96}
#define DEFAULT_MAX_FEEDRATE          {100, 100, 2, 14}    // (mm/sec)    
#define DEFAULT_MAX_ACCELERATION      {2000,2000,30,10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 5.0    // (mm/sec)

//===================================================Optional Feature Settings===========================================//

// Enables auto bed leveling
#define ENABLE_AUTO_BED_LEVELING

// These are the offsets to the prob relative to the extruder tip (Hotend - Probe)
#ifdef ENABLE_AUTO_BED_LEVELING
  #define X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT 25
  #define Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT 0
  #define Z_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT -0.8
#endif

// If defined, the Probe servo will be turned on only during movement and then turned off to avoid jerk
// You MUST HAVE the SERVO_ENDSTOPS defined to use here a value higher than zero otherwise your code will not compile.
//#define PROBE_SERVO_DEACTIVATION_DELAY 300

//If you have enabled the Bed Auto Levelling and are using the same Z Probe for Z Homing.
#define Z_SAFE_HOMING

// With advanced bed leveling, the bed is sampled in a ADVANCED_BED_LEVELING_POINTSxACCURATE_BED_LEVELING_POINTS grid and least squares solution is calculated
// Note: this feature occupies 10'206 byte
//#define ADVANCED_BED_LEVELING

//======================================================Auto Defining Settings==============================================//
// Note: These settings are auto defining and should not need to be changed unless problems occures

// General PID settings
#ifdef PIDTEMP
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID
  #define PID_dT ((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine
#endif

// If ENDSTOPPULLUPS not defined it disables all upllups
#ifndef ENDSTOPPULLUPS
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
#endif

// If ENDSTOPPULLUPS is defined it enables all upllups
#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// Disables max endstops for coreXY
#if defined(COREXY) && !defined(DISABLE_MAX_ENDSTOPS)
  #define DISABLE_MAX_ENDSTOPS
#endif

// Calculates the max length for each axis
#define X_MAX_LENGTH (base_max_pos[0] - base_min_pos[0])
#define Y_MAX_LENGTH (base_max_pos[1] - base_min_pos[1])
#define Z_MAX_LENGTH (base_max_pos[2] - base_min_pos[2])

//Calculates points to be probed
#ifdef ENABLE_AUTO_BED_LEVELING

  //offset inboard from the max print area that will be probed
  #define PROBING_OFFSET 5

  // these are the positions on the bed to do the probing
  #define LEFT_PROBE_BED_POSITION ((X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT + sqrt(pow(X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,2)) + (2 * PROBING_OFFSET))/2)
  #define RIGHT_PROBE_BED_POSITION (X_MAX_LENGTH - (sqrt(pow(X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,2)) - X_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT + (2 * PROBING_OFFSET))/2)
  #define BACK_PROBE_BED_POSITION (Y_MAX_LENGTH - (sqrt(pow(Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,2)) - Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT + (2 * PROBING_OFFSET))/2)
  #define FRONT_PROBE_BED_POSITION ((Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT + sqrt(pow(Y_PROBE_OFFSET_FROM_EXTRUDER_DEFAULT,2)) + (2 * PROBING_OFFSET))/2)

  #define Z_RAISE_BEFORE_HOMING 5       // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case

  #define XY_TRAVEL_SPEED 6000         // X and Y axis travel speed between probes, in mm/min

  #define Z_RAISE_BEFORE_PROBING 5    //How much the extruder will be raised before traveling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 5  //How much the extruder will be raised when traveling from between next probing points
#endif

// If you are using probe for Z homeing
#ifdef Z_SAFE_HOMING
  #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28)
  #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28)
#endif

// Selects the number of accurate bed leveling points
#ifdef ADVANCED_BED_LEVELING
   // I wouldn't see a reason to go above 3 (=9 probing points on the bed)
  #define ACCURATE_BED_LEVELING_POINTS 2
#endif

#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0}  // set the homing speeds (mm/min)

//========================================================Safety Settings===================================================//
// Note: These settings prevent your printer from performing unsafe actions and should not be disabled under normal operation.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
#define PREVENT_DANGEROUS_EXTRUDE
#define EXTRUDE_MINTEMP 170

//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH)


//=========================================================Debug Settings==================================================//
// These settings are for debugging certain 3d printer settings and should not be activated for normal operation

// Define this to have the electronics keep the powersupply off on startup.
// #define PS_DEFAULT_OFF

// Sends debug data to the serial port. (for hot end)
//#define PID_DEBUG
  
// Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
//#define PID_OPENLOOP 1 


