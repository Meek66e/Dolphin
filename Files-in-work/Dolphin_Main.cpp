/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Dolphin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
  #include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING

#define SERVO_LEVELING (defined(ENABLE_AUTO_BED_LEVELING) && PROBE_SERVO_DEACTIVATION_DELAY > 0)

#ifdef MESH_BED_LEVELING
  #include "mesh_bed_leveling.h"
#endif

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "gcodes.h"

#ifdef BLINKM
  #include "BlinkM.h"
  #include "Wire.h"
#endif

#if NUM_SERVOS > 0
  #include "Servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#ifdef SDSUPPORT
  CardReader card;
#endif

float homing_feedrate[] = HOMING_FEEDRATE;
int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extruder_multiply[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100, 100, 100, 100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA);
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0, 1.0, 1.0, 1.0);
float current_position[NUM_AXIS] = { 0.0 };
float home_offset[3] = { 0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = { false };
uint8_t active_extruder = 0;
int fanSpeed = 0;
bool cancel_heatup = false;
const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = { 0 };
static float offset[3] = { 0 };
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; ///< A pointer to find chars in the command string (X, Y, Z, E, etc.)
const char* queued_commands_P= NULL; /* pointer to the current line in the active sequence of commands, or NULL when none */
const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42
// Inactivity shutdown
unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
unsigned long starttime = 0; ///< Print job start time
unsigned long stoptime = 0;  ///< Print job stop time
static uint8_t target_extruder;
bool Stopped = false;
bool CooldownNoWait = true;
bool target_direction;

#ifdef ENABLE_AUTO_BED_LEVELING
  int xy_travel_speed = XY_TRAVEL_SPEED;
  float zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif

#if defined(Z_DUAL_ENDSTOPS) && !defined(DELTA)
  float z_endstop_adj = 0;
#endif

// Extruder offsets
#if EXTRUDERS > 1
  #ifndef EXTRUDER_OFFSET_X
    #define EXTRUDER_OFFSET_X { 0 }
  #endif
  #ifndef EXTRUDER_OFFSET_Y
    #define EXTRUDER_OFFSET_Y { 0 }
  #endif
  float extruder_offset[][EXTRUDERS] = {
    EXTRUDER_OFFSET_X,
    EXTRUDER_OFFSET_Y
    #ifdef DUAL_X_CARRIAGE
      , { 0 } // supports offsets in XYZ plane
    #endif
  };
#endif

#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif

#ifdef BARICUDA
  int ValvePressure = 0;
  int EtoPPressure = 0;
#endif

#ifdef FWRETRACT

  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };

  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if defined(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply = 
    #ifdef PS_DEFAULT_OFF
      false
    #else
      true
    #endif
  ;
#endif

#ifdef DELTA
  float delta[3] = { 0 };
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  float endstop_adj[3] = { 0 };
  // these are the default values, can be overriden with M665
  float delta_radius = DELTA_RADIUS;
  float delta_tower1_x = -SIN_60 * delta_radius; // front left tower
  float delta_tower1_y = -COS_60 * delta_radius;     
  float delta_tower2_x =  SIN_60 * delta_radius; // front right tower
  float delta_tower2_y = -COS_60 * delta_radius;     
  float delta_tower3_x = 0;                      // back middle tower
  float delta_tower3_y = delta_radius;
  float delta_diagonal_rod = DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_2 = sq(delta_diagonal_rod);
  float delta_segments_per_second = DELTA_SEGMENTS_PER_SECOND;
  #ifdef ENABLE_AUTO_BED_LEVELING
    int delta_grid_spacing[2] = { 0, 0 };
    float bed_level[AUTO_BED_LEVELING_GRID_POINTS][AUTO_BED_LEVELING_GRID_POINTS];
  #endif
#else
  static bool home_all_axis = true;
#endif

#ifdef SCARA
  static float delta[3] = { 0 };
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1
#endif

#ifdef FILAMENT_SENSOR
  //Variables for Filament Sensor input
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404
  bool filament_sensor = false;  //M405 turns on filament_sensor control, M406 turns it off
  float filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA; //Stores the measured filament diameter
  signed char measurement_delay[MAX_MEASUREMENT_DELAY+1];  //ring buffer to delay measurement  store extruder factor after subtracting 100
  int delay_index1 = 0;  //index into ring buffer
  int delay_index2 = -1;  //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
  float delay_dist = 0; //delay distance counter
  int meas_delay_cm = MEASUREMENT_DELAY_CM;  //distance delay setting
#endif

#ifdef FILAMENT_RUNOUT_SENSOR
   static bool filrunoutEnqued = false;
#endif

#ifdef SDSUPPORT
  static bool fromsd[BUFSIZE];
#endif

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

#ifdef CHDK
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif //!SDSUPPORT

//Injects the next command from the pending sequence of commands, when possible
//Return false if and only if no command was pending
static bool drain_queued_commands_P() {
  if (!queued_commands_P) return false;

  // Get the next 30 chars from the sequence of gcodes to run
  char cmd[30];
  strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Look for the end of line, or the end of sequence
  size_t i = 0;
  char c;
  while((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
  cmd[i] = '\0';
  if (enquecommand(cmd)) {        // buffer was not full (else we will retry later)
    if (c)
      queued_commands_P += i + 1; // move to next command
    else
      queued_commands_P = NULL;   // will have no more commands in the sequence
  }
  return true;
}

//Record one or many commands to run from program memory.
//Aborts the current queue, if any.
//Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
void enquecommands_P(const char* pgcode) {
    queued_commands_P = pgcode;
    drain_queued_commands_P(); // first command executed asap (when possible)
}

//adds a single command to the main command buffer, from RAM
//that is really done in a non-safe way.
//needs overworking someday
//Returns false if it failed to do so
bool enquecommand(const char *cmd)
{
  if(*cmd==';')
    return false;
  if(buflen >= BUFSIZE)
    return false;
  //this is dangerous if a mixing of serial and this happens
  strcpy(&(cmdbuffer[bufindw][0]),cmd);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_Enqueing);
  SERIAL_ECHO(cmdbuffer[bufindw]);
  SERIAL_ECHOLNPGM("\"");
  bufindw= (bufindw + 1)%BUFSIZE;
  buflen += 1;
  return true;
}

void setup_killpin()
{
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  #endif
}

void setup_filrunoutpin()
{
  #if HAS_FILRUNOUT
    pinMode(FILRUNOUT_PIN, INPUT);
    #ifdef ENDSTOPPULLUP_FIL_RUNOUT
      WRITE(FILLRUNOUT_PIN, HIGH);
    #endif
  #endif
}

// Set home pin
void setup_homepin(void)
{
  #if HAS_HOME
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  #endif
}


void setup_photpin()
{
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #ifdef PS_DEFAULT_OFF
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide()
{
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servos[0].attach(SERVO0_PIN);
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servos[1].attach(SERVO1_PIN);
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servos[2].attach(SERVO2_PIN);
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servos[3].attach(SERVO3_PIN);
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for (int i = 0; i < 3; i++)
    if (servo_endstops[i] >= 0)
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
  #endif

  #if SERVO_LEVELING
    delay(PROBE_SERVO_DEACTIVATION_DELAY);
    servos[servo_endstops[Z_AXIS]].detach();
  #endif
}


void setup()
{
  setup_killpin();
  setup_filrunoutpin();
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(STRING_VERSION);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_VERSION_CONFIG_H
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  #ifdef SDSUPPORT
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }
  #endif //!SDSUPPORT

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();
  

  lcd_init();
  _delay_ms(1000);  // wait 1sec to display the splash screen

  #if HAS_CONTROLLERFAN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif
#ifdef Z_PROBE_SLED
  pinMode(SERVO0_PIN, OUTPUT);
  digitalWrite(SERVO0_PIN, LOW); // turn it off
#endif // Z_PROBE_SLED
  setup_homepin();
  
#ifdef STAT_LED_RED
  pinMode(STAT_LED_RED, OUTPUT);
  digitalWrite(STAT_LED_RED, LOW); // turn it off
#endif
#ifdef STAT_LED_BLUE
  pinMode(STAT_LED_BLUE, OUTPUT);
  digitalWrite(STAT_LED_BLUE, LOW); // turn it off
#endif  
}


void loop() {
  if (buflen < BUFSIZE - 1) get_command();

  #ifdef SDSUPPORT
    card.checkautostart(false);
  #endif

  if (buflen) {
    #ifdef SDSUPPORT
      if (card.saving) {
        if (strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL) {
          card.write_command(cmdbuffer[bufindr]);
          if (card.logging)
            process_commands();
          else
            SERIAL_PROTOCOLLNPGM(MSG_OK);
        }
        else {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }
      }
      else
        process_commands();
    #else
      process_commands();
    #endif // SDSUPPORT
    buflen--;
    bufindr = (bufindr + 1) % BUFSIZE;
  }
  // Check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
}

void get_command()
{
  if (drain_queued_commands_P()) // priority is given to non-serial commands
    return;
  
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      // end of line == end of comment
      comment_mode = false;

      if(!serial_count) {
        // short cut for empty lines
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      #ifdef SDSUPPORT
      fromsd[bufindw] = false;
      #endif //!SDSUPPORT
      if(strchr(cmdbuffer[bufindw], 'N') != NULL)
      {
        strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
        gcode_N = (strtol(strchr_pointer + 1, NULL, 10));
        if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
          SERIAL_ERRORLN(gcode_LastN);
          //Serial.println(gcode_N);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        if(strchr(cmdbuffer[bufindw], '*') != NULL)
        {
          byte checksum = 0;
          byte count = 0;
          while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
          strchr_pointer = strchr(cmdbuffer[bufindw], '*');

          if(strtol(strchr_pointer + 1, NULL, 10) != checksum) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          //if no errors, continue parsing
        }
        else
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
          SERIAL_ERRORLN(gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        gcode_LastN = gcode_N;
        //if no errors, continue parsing
      }
      else  // if we don't receive 'N' but still see '*'
      {
        if((strchr(cmdbuffer[bufindw], '*') != NULL))
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
          SERIAL_ERRORLN(gcode_LastN);
          serial_count = 0;
          return;
        }
      }
      if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
        strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
        switch(strtol(strchr_pointer + 1, NULL, 10)){
        case 0:
        case 1:
        case 2:
        case 3:
          if (Stopped == true) {
            SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            LCD_MESSAGEPGM(MSG_STOPPED);
          }
          break;
        default:
          break;
        }

      }

      //If command was e-stop process now
      if(strcmp(cmdbuffer[bufindw], "M112") == 0)
        kill();

      bufindw = (bufindw + 1)%BUFSIZE;
      buflen += 1;

      serial_count = 0; //clear buffer
    }
    else if(serial_char == '\\') {  //Handle escapes
       
        if(MYSERIAL.available() > 0  && buflen < BUFSIZE) {
            // if we have one more character, copy it over
            serial_char = MYSERIAL.read();
            cmdbuffer[bufindw][serial_count++] = serial_char;
        }

        //otherwise do nothing        
    }
    else { // its not a newline, carriage return or escape char
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  #ifdef SDSUPPORT
  if(!card.sdprinting || serial_count!=0){
    return;
  }

  //'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
  // if it occurs, stop_buffering is triggered and the buffer is ran dry.
  // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

  static bool stop_buffering=false;
  if(buflen==0) stop_buffering=false;

  while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == '#' && comment_mode == false) ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i "MSG_END_HOUR" %i "MSG_END_MINUTE),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        lcd_setstatus(time, true);
        card.printingHasFinished();
        card.checkautostart(true);

      }
      if(serial_char=='#')
        stop_buffering=true;

      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #endif //SDSUPPORT

}

float code_value() {
  float ret;
  char *e = strchr(strchr_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(strchr_pointer+1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(strchr_pointer+1, NULL);
  return ret;
}

long code_value_long() { return strtol(strchr_pointer + 1, NULL, 10); }

int16_t code_value_short() { return (int16_t)strtol(strchr_pointer + 1, NULL, 10); }

bool code_seen(char code) {
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm, HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE

  #define DXC_FULL_CONTROL_MODE 0
  #define DXC_AUTO_PARK_MODE    1
  #define DXC_DUPLICATION_MODE  2

  static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  static float x_home_pos(int extruder) {
    if (extruder == 0)
      return base_home_pos(X_AXIS) + home_offset[X_AXIS];
    else
      // In dual carriage mode the extruder offset provides an override of the
      // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
      // This allow soft recalibration of the second extruder offset position without firmware reflash
      // (through the M218 command).
      return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
  }

  static int x_home_dir(int extruder) {
    return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
  }

  static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
  static bool active_extruder_parked = false; // used in mode 1 & 2
  static float raised_parked_position[NUM_AXIS]; // used in mode 1
  static unsigned long delayed_move_time = 0; // used in mode 1
  static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_extruder_temp_offset = 0; // used in mode 2
  bool extruder_duplication_enabled = false; // used in mode 2

#endif //DUAL_X_CARRIAGE

static void axis_is_at_home(int axis) {

  #ifdef DUAL_X_CARRIAGE
    if (axis == X_AXIS) {
      if (active_extruder != 0) {
        current_position[X_AXIS] = x_home_pos(active_extruder);
                 min_pos[X_AXIS] = X2_MIN_POS;
                 max_pos[X_AXIS] = max(extruder_offset[X_AXIS][1], X2_MAX_POS);
        return;
      }
      else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
        float xoff = home_offset[X_AXIS];
        current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;
                 min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
                 max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
        return;
      }
    }
  #endif

  #ifdef SCARA
    float homeposition[3];
   
    if (axis < 2) {

      for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);

      // SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
      // SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);
      // Works out real Homeposition angles using inverse kinematics, 
      // and calculates homing offset using forward kinematics
      calculate_delta(homeposition);
     
      // SERIAL_ECHOPGM("base Theta= "); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" base Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);
     
      for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];
     
      // SERIAL_ECHOPGM("addhome X="); SERIAL_ECHO(home_offset[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Y="); SERIAL_ECHO(home_offset[Y_AXIS]);
      // SERIAL_ECHOPGM(" addhome Theta="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);
      
      calculate_SCARA_forward_Transform(delta);
     
      // SERIAL_ECHOPGM("Delta X="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);
     
      current_position[axis] = delta[axis];
    
      // SCARA home positions are based on configuration since the actual limits are determined by the 
      // inverse kinematic transform.
      min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
      max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
    } 
    else {
      current_position[axis] = base_home_pos(axis) + home_offset[axis];
      min_pos[axis] = base_min_pos(axis) + home_offset[axis];
      max_pos[axis] = base_max_pos(axis) + home_offset[axis];
    }
  #else
    current_position[axis] = base_home_pos(axis) + home_offset[axis];
    min_pos[axis] = base_min_pos(axis) + home_offset[axis];
    max_pos[axis] = base_max_pos(axis) + home_offset[axis];
  #endif
}

/**
 * Some planner shorthand inline functions
 */
inline void line_to_current_position() {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
}
inline void line_to_z(float zPosition) {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
}
inline void line_to_destination() {
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
}
inline void sync_plan_position() {
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#if defined(DELTA) || defined(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

#ifdef ENABLE_AUTO_BED_LEVELING

  #ifdef DELTA
    /**
     * Calculate delta, start a line, and set current_position to destination
     */
    void prepare_move_raw() {
      refresh_cmd_timeout();
      calculate_delta(destination);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], (feedrate/60)*(feedmultiply/100.0), active_extruder);
      set_current_to_destination();
    }
  #endif

  #ifdef AUTO_BED_LEVELING_GRID

    #ifndef DELTA

      static void set_bed_level_equation_lsq(double *plane_equation_coefficients) {
        vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
        planeNormal.debug("planeNormal");
        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
        //bedLevel.debug("bedLevel");

        //plan_bed_level_matrix.debug("bed level before");
        //vector_3 uncorrected_position = plan_get_position_mm();
        //uncorrected_position.debug("position before");

        vector_3 corrected_position = plan_get_position();
        //corrected_position.debug("position after");
        current_position[X_AXIS] = corrected_position.x;
        current_position[Y_AXIS] = corrected_position.y;
        current_position[Z_AXIS] = corrected_position.z;

        sync_plan_position();
      }

    #endif // !DELTA

  #else // !AUTO_BED_LEVELING_GRID

    static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

      plan_bed_level_matrix.set_to_identity();

      vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
      vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
      vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
      vector_3 planeNormal = vector_3::cross(pt1 - pt2, pt3 - pt2).get_normal();

      if (planeNormal.z < 0) {
        planeNormal.x = -planeNormal.x;
        planeNormal.y = -planeNormal.y;
        planeNormal.z = -planeNormal.z;
      }

      plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

      vector_3 corrected_position = plan_get_position();
      current_position[X_AXIS] = corrected_position.x;
      current_position[Y_AXIS] = corrected_position.y;
      current_position[Z_AXIS] = corrected_position.z;

      sync_plan_position();
    }

  #endif // !AUTO_BED_LEVELING_GRID

  static void run_z_probe() {

    #ifdef DELTA
    
      float start_z = current_position[Z_AXIS];
      long start_steps = st_get_position(Z_AXIS);
    
      // move down slowly until you find the bed
      feedrate = homing_feedrate[Z_AXIS] / 4;
      destination[Z_AXIS] = -10;
      prepare_move_raw();
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags
      
      // we have to let the planner know where we are right now as it is not where we said to go.
      long stop_steps = st_get_position(Z_AXIS);
      float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
      current_position[Z_AXIS] = mm;
      sync_plan_position_delta();
      
    #else // !DELTA

      plan_bed_level_matrix.set_to_identity();
      feedrate = homing_feedrate[Z_AXIS];

      // move down until you find the bed
      float zPosition = -10;
      line_to_z(zPosition);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      zPosition = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

      // move up the retract distance
      zPosition += home_bump_mm(Z_AXIS);
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      // move back down slowly to find bed
      if (homing_bump_divisor[Z_AXIS] >= 1)
        feedrate = homing_feedrate[Z_AXIS] / homing_bump_divisor[Z_AXIS];
      else {
        feedrate = homing_feedrate[Z_AXIS] / 10;
        SERIAL_ECHOLN("Warning: The Homing Bump Feedrate Divisor cannot be less than 1");
      }

      zPosition -= home_bump_mm(Z_AXIS) * 2;
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      // make sure the planner knows where we are as it may be a bit different than we last said to move to
      sync_plan_position();
      
    #endif // !DELTA
  }

  /**
   * 
   */
  static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    #ifdef DELTA

      feedrate = XY_TRAVEL_SPEED;
      
      destination[X_AXIS] = x;
      destination[Y_AXIS] = y;
      destination[Z_AXIS] = z;
      prepare_move_raw();
      st_synchronize();

    #else

      feedrate = homing_feedrate[Z_AXIS];

      current_position[Z_AXIS] = z;
      line_to_current_position();
      st_synchronize();

      feedrate = xy_travel_speed;

      current_position[X_AXIS] = x;
      current_position[Y_AXIS] = y;
      line_to_current_position();
      st_synchronize();

    #endif

    feedrate = oldFeedRate;
  }

  static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    refresh_cmd_timeout();
    enable_endstops(true);
  }

  static void clean_up_after_endstop_move() {
    #ifdef ENDSTOPS_ONLY_FOR_HOMING
      enable_endstops(false);
    #endif
    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    refresh_cmd_timeout();
  }

  static void deploy_z_probe() {

    #ifdef SERVO_ENDSTOPS

      // Engage Z Servo endstop if enabled
      if (servo_endstops[Z_AXIS] >= 0) {
        #if SERVO_LEVELING
          servos[servo_endstops[Z_AXIS]].attach(0);
        #endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
        #if SERVO_LEVELING
          delay(PROBE_SERVO_DEACTIVATION_DELAY);
          servos[servo_endstops[Z_AXIS]].detach();
        #endif
      }

    #elif defined(Z_PROBE_ALLEN_KEY)

      feedrate = homing_feedrate[X_AXIS];

      // Move to the start position to initiate deployment
      destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_X;
      destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_Y;
      destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_Z;
      prepare_move_raw();

      // Home X to touch the belt
      feedrate = homing_feedrate[X_AXIS]/10;
      destination[X_AXIS] = 0;
      prepare_move_raw();
      
      // Home Y for safety
      feedrate = homing_feedrate[X_AXIS]/2;
      destination[Y_AXIS] = 0;
      prepare_move_raw();
      
      st_synchronize();

    #ifdef Z_PROBE_ENDSTOP
      bool z_probe_endstop = (READ(Z_PROBE_PIN) != Z_PROBE_ENDSTOP_INVERTING);
      if (z_probe_endstop)
    #else
      bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
      if (z_min_endstop)
    #endif
      {
        if (!Stopped) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Z-Probe failed to engage!");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        Stop();
      }

    #endif // Z_PROBE_ALLEN_KEY

  }

  static void stow_z_probe() {

    #ifdef SERVO_ENDSTOPS

      // Retract Z Servo endstop if enabled
      if (servo_endstops[Z_AXIS] >= 0) {

        #if Z_RAISE_AFTER_PROBING > 0
          do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING);
          st_synchronize();
        #endif

        #if SERVO_LEVELING
          servos[servo_endstops[Z_AXIS]].attach(0);
        #endif

        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);

        #if SERVO_LEVELING
          delay(PROBE_SERVO_DEACTIVATION_DELAY);
          servos[servo_endstops[Z_AXIS]].detach();
        #endif
      }

    #elif defined(Z_PROBE_ALLEN_KEY)

      // Move up for safety
      feedrate = homing_feedrate[X_AXIS];
      destination[Z_AXIS] = current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING;
      prepare_move_raw();

      // Move to the start position to initiate retraction
      destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_X;
      destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_Y;
      destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_Z;
      prepare_move_raw();

      // Move the nozzle down to push the probe into retracted position
      feedrate = homing_feedrate[Z_AXIS]/10;
      destination[Z_AXIS] = current_position[Z_AXIS] - Z_PROBE_ALLEN_KEY_STOW_DEPTH;
      prepare_move_raw();
      
      // Move up for safety
      feedrate = homing_feedrate[Z_AXIS]/2;
      destination[Z_AXIS] = current_position[Z_AXIS] + Z_PROBE_ALLEN_KEY_STOW_DEPTH * 2;
      prepare_move_raw();
      
      // Home XY for safety
      feedrate = homing_feedrate[X_AXIS]/2;
      destination[X_AXIS] = 0;
      destination[Y_AXIS] = 0;
      prepare_move_raw();
      
      st_synchronize();

    #ifdef Z_PROBE_ENDSTOP
      bool z_probe_endstop = (READ(Z_PROBE_PIN) != Z_PROBE_ENDSTOP_INVERTING);
      if (!z_probe_endstop)
    #else
      bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
      if (!z_min_endstop)
    #endif
      {
        if (!Stopped) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Z-Probe failed to retract!");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        Stop();
      }

    #endif

  }

  enum ProbeAction {
    ProbeStay          = 0,
    ProbeDeploy        = BIT(0),
    ProbeStow          = BIT(1),
    ProbeDeployAndStow = (ProbeDeploy | ProbeStow)
  };

  // Probe bed height at position (x,y), returns the measured z value
  static float probe_pt(float x, float y, float z_before, ProbeAction retract_action=ProbeDeployAndStow, int verbose_level=1) {
    // move to right place
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
    do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

    #if !defined(Z_PROBE_SLED) && !defined(Z_PROBE_ALLEN_KEY)
      if (retract_action & ProbeDeploy) deploy_z_probe();
    #endif

    run_z_probe();
    float measured_z = current_position[Z_AXIS];

    #if Z_RAISE_BETWEEN_PROBINGS > 0
      if (retract_action == ProbeStay) {
        do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
        st_synchronize();
      }
    #endif

    #if !defined(Z_PROBE_SLED) && !defined(Z_PROBE_ALLEN_KEY)
      if (retract_action & ProbeStow) stow_z_probe();
    #endif

    if (verbose_level > 2) {
      SERIAL_PROTOCOLPGM(MSG_BED);
      SERIAL_PROTOCOLPGM(" X: ");
      SERIAL_PROTOCOL_F(x, 3);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL_F(y, 3);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL_F(measured_z, 3);
      SERIAL_EOL;
    }
    return measured_z;
  }

  #ifdef DELTA

    /**
     * All DELTA leveling in the Marlin uses NONLINEAR_BED_LEVELING
     */

    static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
      if (bed_level[x][y] != 0.0) {
        return;  // Don't overwrite good values.
      }
      float a = 2*bed_level[x+xdir][y] - bed_level[x+xdir*2][y];  // Left to right.
      float b = 2*bed_level[x][y+ydir] - bed_level[x][y+ydir*2];  // Front to back.
      float c = 2*bed_level[x+xdir][y+ydir] - bed_level[x+xdir*2][y+ydir*2];  // Diagonal.
      float median = c;  // Median is robust (ignores outliers).
      if (a < b) {
        if (b < c) median = b;
        if (c < a) median = a;
      } else {  // b <= a
        if (c < b) median = b;
        if (a < c) median = a;
      }
      bed_level[x][y] = median;
    }

    // Fill in the unprobed points (corners of circular print surface)
    // using linear extrapolation, away from the center.
    static void extrapolate_unprobed_bed_level() {
      int half = (AUTO_BED_LEVELING_GRID_POINTS-1)/2;
      for (int y = 0; y <= half; y++) {
        for (int x = 0; x <= half; x++) {
          if (x + y < 3) continue;
          extrapolate_one_point(half-x, half-y, x>1?+1:0, y>1?+1:0);
          extrapolate_one_point(half+x, half-y, x>1?-1:0, y>1?+1:0);
          extrapolate_one_point(half-x, half+y, x>1?+1:0, y>1?-1:0);
          extrapolate_one_point(half+x, half+y, x>1?-1:0, y>1?-1:0);
        }
      }
    }

    // Print calibration results for plotting or manual frame adjustment.
    static void print_bed_level() {
      for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
          SERIAL_PROTOCOL_F(bed_level[x][y], 2);
          SERIAL_PROTOCOLCHAR(' ');
        }
        SERIAL_EOL;
      }
    }

    // Reset calibration results to zero.
    void reset_bed_level() {
      for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
          bed_level[x][y] = 0.0;
        }
      }
    }

  #endif // DELTA

#endif // ENABLE_AUTO_BED_LEVELING

/**
 * Home an individual axis
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(int axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

    int axis_home_dir;

    #ifdef DUAL_X_CARRIAGE
      if (axis == X_AXIS) axis_home_dir = x_home_dir(active_extruder);
    #else
      axis_home_dir = home_dir(axis);
    #endif

    // Set the axis position as setup for the move
    current_position[axis] = 0;
    sync_plan_position();

    // Engage Servo endstop if enabled
    #if defined(SERVO_ENDSTOPS) && !defined(Z_PROBE_SLED)

      #if SERVO_LEVELING
        if (axis == Z_AXIS) deploy_z_probe(); else
      #endif
        {
          if (servo_endstops[axis] > -1)
            servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
        }

    #endif // SERVO_ENDSTOPS && !Z_PROBE_SLED

    #ifdef Z_DUAL_ENDSTOPS
      if (axis == Z_AXIS) In_Homing_Process(true);
    #endif

    // Move towards the endstop until an endstop is triggered
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    line_to_destination();
    st_synchronize();

    // Set the axis position as setup for the move
    current_position[axis] = 0;
    sync_plan_position();

    // Move away from the endstop by the axis HOME_BUMP_MM
    destination[axis] = -home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    // Slow down the feedrate for the next move
    if (homing_bump_divisor[axis] >= 1)
      feedrate = homing_feedrate[axis] / homing_bump_divisor[axis];
    else {
      feedrate = homing_feedrate[axis] / 10;
      SERIAL_ECHOLNPGM("Warning: The Homing Bump Feedrate Divisor cannot be less than 1");
    }

    // Move slowly towards the endstop until triggered
    destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    #ifdef Z_DUAL_ENDSTOPS
      if (axis == Z_AXIS) {
        float adj = fabs(z_endstop_adj);
        bool lockZ1;
        if (axis_home_dir > 0) {
          adj = -adj;
          lockZ1 = (z_endstop_adj > 0);
        }
        else
          lockZ1 = (z_endstop_adj < 0);

        if (lockZ1) Lock_z_motor(true); else Lock_z2_motor(true);
        sync_plan_position();

        // Move to the adjusted endstop height
        feedrate = homing_feedrate[axis];
        destination[Z_AXIS] = adj;
        line_to_destination();
        st_synchronize();

        if (lockZ1) Lock_z_motor(false); else Lock_z2_motor(false);
        In_Homing_Process(false);
      } // Z_AXIS
    #endif

    #ifdef DELTA
      // retrace by the amount specified in endstop_adj
      if (endstop_adj[axis] * axis_home_dir < 0) {
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        line_to_destination();
        st_synchronize();
      }
    #endif

    // Set the axis position to its home position (plus home offsets)
    axis_is_at_home(axis);

    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose(); // clear endstop hit flags
    axis_known_position[axis] = true;

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1)
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
    #endif

    #if SERVO_LEVELING && !defined(Z_PROBE_SLED)
      if (axis == Z_AXIS) stow_z_probe();
    #endif

  }
}

#ifdef FWRETRACT

  void retract(bool retracting, bool swapretract = false) {

    if (retracting == retracted[active_extruder]) return;

    float oldFeedrate = feedrate;

    set_destination_to_current();

    if (retracting) {

      feedrate = retract_feedrate * 60;
      current_position[E_AXIS] += (swapretract ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        prepare_move();
      }
    }
    else {

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] += retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        //prepare_move();
      }

      feedrate = retract_recover_feedrate * 60;
      float move_e = swapretract ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
    }

    feedrate = oldFeedrate;
    retracted[active_extruder] = retract;

  } // retract()

#endif // FWRETRACT

#ifdef Z_PROBE_SLED

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  //
  // Method to dock/undock a sled designed by Charles Bell.
  //
  // dock[in]     If true, move to MAX_X and engage the electromagnet
  // offset[in]   The additional distance to move to adjust docking location
  //
  static void dock_sled(bool dock, int offset=0) {
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    if (dock) {
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, current_position[Y_AXIS], current_position[Z_AXIS]);
      digitalWrite(SERVO0_PIN, LOW); // turn off magnet
    } else {
      float z_loc = current_position[Z_AXIS];
      if (z_loc < Z_RAISE_BEFORE_PROBING + 5) z_loc = Z_RAISE_BEFORE_PROBING;
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc);
      digitalWrite(SERVO0_PIN, HIGH); // turn on magnet
    }
  }

#endif // Z_PROBE_SLED

void FlushSerialRequestResend() {
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend() {
  refresh_cmd_timeout();
  #ifdef SDSUPPORT
    if (fromsd[bufindr]) return;
  #endif
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates() {
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value() + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }
  if (code_seen('F')) {
    next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates() {
  #ifdef SF_ARC_FIX
    bool relative_mode_backup = relative_mode;
    relative_mode = true;
  #endif
    get_coordinates();
  #ifdef SF_ARC_FIX
    relative_mode = relative_mode_backup;
  #endif

  offset[0] = code_seen('I') ? code_value() : 0;
  offset[1] = code_seen('J') ? code_value() : 0;
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    
    float negative_z_offset = 0;
    #ifdef ENABLE_AUTO_BED_LEVELING
      if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset = negative_z_offset + Z_PROBE_OFFSET_FROM_EXTRUDER;
      if (home_offset[Z_AXIS] < 0) negative_z_offset = negative_z_offset + home_offset[Z_AXIS];
    #endif
    
    if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

#ifdef DELTA

  void recalc_delta_settings(float radius, float diagonal_rod) {
    delta_tower1_x = -SIN_60 * radius;  // front left tower
    delta_tower1_y = -COS_60 * radius;
    delta_tower2_x =  SIN_60 * radius;  // front right tower
    delta_tower2_y = -COS_60 * radius;
    delta_tower3_x = 0.0;               // back middle tower
    delta_tower3_y = radius;
    delta_diagonal_rod_2 = sq(diagonal_rod);
  }

  void calculate_delta(float cartesian[3]) {
    delta[X_AXIS] = sqrt(delta_diagonal_rod_2
                         - sq(delta_tower1_x-cartesian[X_AXIS])
                         - sq(delta_tower1_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
                         - sq(delta_tower2_x-cartesian[X_AXIS])
                         - sq(delta_tower2_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
                         - sq(delta_tower3_x-cartesian[X_AXIS])
                         - sq(delta_tower3_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    /*
    SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

    SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
    SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
    SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
    */
  }

  #ifdef ENABLE_AUTO_BED_LEVELING

    // Adjust print surface height by linear interpolation over the bed_level array.
    void adjust_delta(float cartesian[3]) {
      if (delta_grid_spacing[0] == 0 || delta_grid_spacing[1] == 0) return; // G29 not done!

      int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
      float h1 = 0.001 - half, h2 = half - 0.001,
            grid_x = max(h1, min(h2, cartesian[X_AXIS] / delta_grid_spacing[0])),
            grid_y = max(h1, min(h2, cartesian[Y_AXIS] / delta_grid_spacing[1]));
      int floor_x = floor(grid_x), floor_y = floor(grid_y);
      float ratio_x = grid_x - floor_x, ratio_y = grid_y - floor_y,
            z1 = bed_level[floor_x + half][floor_y + half],
            z2 = bed_level[floor_x + half][floor_y + half + 1],
            z3 = bed_level[floor_x + half + 1][floor_y + half],
            z4 = bed_level[floor_x + half + 1][floor_y + half + 1],
            left = (1 - ratio_y) * z1 + ratio_y * z2,
            right = (1 - ratio_y) * z3 + ratio_y * z4,
            offset = (1 - ratio_x) * left + ratio_x * right;

      delta[X_AXIS] += offset;
      delta[Y_AXIS] += offset;
      delta[Z_AXIS] += offset;

      /*
      SERIAL_ECHOPGM("grid_x="); SERIAL_ECHO(grid_x);
      SERIAL_ECHOPGM(" grid_y="); SERIAL_ECHO(grid_y);
      SERIAL_ECHOPGM(" floor_x="); SERIAL_ECHO(floor_x);
      SERIAL_ECHOPGM(" floor_y="); SERIAL_ECHO(floor_y);
      SERIAL_ECHOPGM(" ratio_x="); SERIAL_ECHO(ratio_x);
      SERIAL_ECHOPGM(" ratio_y="); SERIAL_ECHO(ratio_y);
      SERIAL_ECHOPGM(" z1="); SERIAL_ECHO(z1);
      SERIAL_ECHOPGM(" z2="); SERIAL_ECHO(z2);
      SERIAL_ECHOPGM(" z3="); SERIAL_ECHO(z3);
      SERIAL_ECHOPGM(" z4="); SERIAL_ECHO(z4);
      SERIAL_ECHOPGM(" left="); SERIAL_ECHO(left);
      SERIAL_ECHOPGM(" right="); SERIAL_ECHO(right);
      SERIAL_ECHOPGM(" offset="); SERIAL_ECHOLN(offset);
      */
    }
  #endif // ENABLE_AUTO_BED_LEVELING

#endif // DELTA

#ifdef MESH_BED_LEVELING

  #if !defined(MIN)
    #define MIN(_v1, _v2) (((_v1) < (_v2)) ? (_v1) : (_v2))
  #endif  // ! MIN

// This function is used to split lines on mesh borders so each segment is only part of one mesh area
void mesh_plan_buffer_line(float x, float y, float z, const float e, float feed_rate, const uint8_t &extruder, uint8_t x_splits=0xff, uint8_t y_splits=0xff)
{
  if (!mbl.active) {
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  int pix = mbl.select_x_index(current_position[X_AXIS]);
  int piy = mbl.select_y_index(current_position[Y_AXIS]);
  int ix = mbl.select_x_index(x);
  int iy = mbl.select_y_index(y);
  pix = MIN(pix, MESH_NUM_X_POINTS-2);
  piy = MIN(piy, MESH_NUM_Y_POINTS-2);
  ix = MIN(ix, MESH_NUM_X_POINTS-2);
  iy = MIN(iy, MESH_NUM_Y_POINTS-2);
  if (pix == ix && piy == iy) {
    // Start and end on same mesh square
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  float nx, ny, ne, normalized_dist;
  if (ix > pix && (x_splits) & BIT(ix)) {
    nx = mbl.get_x(ix);
    normalized_dist = (nx - current_position[X_AXIS])/(x - current_position[X_AXIS]);
    ny = current_position[Y_AXIS] + (y - current_position[Y_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    x_splits ^= BIT(ix);
  } else if (ix < pix && (x_splits) & BIT(pix)) {
    nx = mbl.get_x(pix);
    normalized_dist = (nx - current_position[X_AXIS])/(x - current_position[X_AXIS]);
    ny = current_position[Y_AXIS] + (y - current_position[Y_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    x_splits ^= BIT(pix);
  } else if (iy > piy && (y_splits) & BIT(iy)) {
    ny = mbl.get_y(iy);
    normalized_dist = (ny - current_position[Y_AXIS])/(y - current_position[Y_AXIS]);
    nx = current_position[X_AXIS] + (x - current_position[X_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    y_splits ^= BIT(iy);
  } else if (iy < piy && (y_splits) & BIT(piy)) {
    ny = mbl.get_y(piy);
    normalized_dist = (ny - current_position[Y_AXIS])/(y - current_position[Y_AXIS]);
    nx = current_position[X_AXIS] + (x - current_position[X_AXIS]) * normalized_dist;
    ne = current_position[E_AXIS] + (e - current_position[E_AXIS]) * normalized_dist;
    y_splits ^= BIT(piy);
  } else {
    // Already split on a border
    plan_buffer_line(x, y, z, e, feed_rate, extruder);
    set_current_to_destination();
    return;
  }
  // Do the split and look for more borders
  destination[X_AXIS] = nx;
  destination[Y_AXIS] = ny;
  destination[E_AXIS] = ne;
  mesh_plan_buffer_line(nx, ny, z, ne, feed_rate, extruder, x_splits, y_splits);
  destination[X_AXIS] = x;
  destination[Y_AXIS] = y;
  destination[E_AXIS] = e;
  mesh_plan_buffer_line(x, y, z, e, feed_rate, extruder, x_splits, y_splits);
}
#endif  // MESH_BED_LEVELING

void prepare_move() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();
  
  #ifdef SCARA //for now same as delta-code

    float difference[NUM_AXIS];
    for (int8_t i = 0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(  sq(difference[X_AXIS]) +
                                sq(difference[Y_AXIS]) +
                                sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
    if (cartesian_mm < 0.000001) { return; }
    float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
    int steps = max(1, int(scara_segments_per_second * seconds));

    //SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    //SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    //SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for(int8_t i = 0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i] + difference[i] * fraction;
      }
  
      calculate_delta(destination);
      //SERIAL_ECHOPGM("destination[X_AXIS]="); SERIAL_ECHOLN(destination[X_AXIS]);
      //SERIAL_ECHOPGM("destination[Y_AXIS]="); SERIAL_ECHOLN(destination[Y_AXIS]);
      //SERIAL_ECHOPGM("destination[Z_AXIS]="); SERIAL_ECHOLN(destination[Z_AXIS]);
      //SERIAL_ECHOPGM("delta[X_AXIS]="); SERIAL_ECHOLN(delta[X_AXIS]);
      //SERIAL_ECHOPGM("delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
      //SERIAL_ECHOPGM("delta[Z_AXIS]="); SERIAL_ECHOLN(delta[Z_AXIS]);

      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
        destination[E_AXIS], feedrate*feedmultiply/60/100.0,
        active_extruder);
    }

  #endif // SCARA
  
  #ifdef DELTA

    float difference[NUM_AXIS];
    for (int8_t i=0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                              sq(difference[Y_AXIS]) +
                              sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) cartesian_mm = abs(difference[E_AXIS]);
    if (cartesian_mm < 0.000001) return;
    float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
    int steps = max(1, int(delta_segments_per_second * seconds));

    // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for (int8_t i = 0; i < NUM_AXIS; i++) destination[i] = current_position[i] + difference[i] * fraction;
      calculate_delta(destination);
      #ifdef ENABLE_AUTO_BED_LEVELING
        adjust_delta(destination);
      #endif
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                       destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                       active_extruder);
    }

  #endif // DELTA

  #ifdef DUAL_X_CARRIAGE
    if (active_extruder_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS],
            current_position[E_AXIS], max_feedrate[X_AXIS], 1);
        sync_plan_position();
        st_synchronize();
        extruder_duplication_enabled = true;
        active_extruder_parked = false;
      }
      else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) { // handle unparking of head
        if (current_position[E_AXIS] == destination[E_AXIS]) {
          // this is a travel move - skit it but keep track of current position (so that it can later
          // be used as start of first non-travel move)
          if (delayed_move_time != 0xFFFFFFFFUL) {
            set_current_to_destination();
            if (destination[Z_AXIS] > raised_parked_position[Z_AXIS])
              raised_parked_position[Z_AXIS] = destination[Z_AXIS];
            delayed_move_time = millis();
            return;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
        plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS],    current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS],
            current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
            current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
        active_extruder_parked = false;
      }
    }
  #endif // DUAL_X_CARRIAGE

  #if !defined(DELTA) && !defined(SCARA)
    // Do not use feedmultiply for E or Z only moves
    if ( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      line_to_destination();
    }
    else {
      #ifdef MESH_BED_LEVELING
        mesh_plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], (feedrate/60)*(feedmultiply/100.0), active_extruder);
        return;
      #else
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], (feedrate/60)*(feedmultiply/100.0), active_extruder);
      #endif  // MESH_BED_LEVELING
    }
  #endif // !(DELTA || SCARA)

  set_current_to_destination();
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  set_current_to_destination();
  refresh_cmd_timeout();
}

#if HAS_CONTROLLERFAN

unsigned long lastMotor = 0; // Last time a motor was turned on
unsigned long lastMotorCheck = 0; // Last time the state was checked

void controllerFan() {
  uint32_t ms = millis();
  if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms
    lastMotorCheck = ms;
    if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
      || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
      #if EXTRUDERS > 1
        || E1_ENABLE_READ == E_ENABLE_ON
        #if HAS_X2_ENABLE
          || X2_ENABLE_READ == X_ENABLE_ON
        #endif
        #if EXTRUDERS > 2
          || E2_ENABLE_READ == E_ENABLE_ON
          #if EXTRUDERS > 3
            || E3_ENABLE_READ == E_ENABLE_ON
          #endif
        #endif
      #endif
    ) {
      lastMotor = ms; //... set time to NOW so the fan will turn on
    }
    uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
    // allows digital or PWM fan output to be used (see M42 handling)
    digitalWrite(CONTROLLERFAN_PIN, speed);
    analogWrite(CONTROLLERFAN_PIN, speed);
  }
}
#endif

#ifdef SCARA
void calculate_SCARA_forward_Transform(float f_scara[3])
{
  // Perform forward kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float x_sin, x_cos, y_sin, y_cos;
  
    //SERIAL_ECHOPGM("f_delta x="); SERIAL_ECHO(f_scara[X_AXIS]);
    //SERIAL_ECHOPGM(" y="); SERIAL_ECHO(f_scara[Y_AXIS]);
  
    x_sin = sin(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    x_cos = cos(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    y_sin = sin(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
    y_cos = cos(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
   
  //  SERIAL_ECHOPGM(" x_sin="); SERIAL_ECHO(x_sin);
  //  SERIAL_ECHOPGM(" x_cos="); SERIAL_ECHO(x_cos);
  //  SERIAL_ECHOPGM(" y_sin="); SERIAL_ECHO(y_sin);
  //  SERIAL_ECHOPGM(" y_cos="); SERIAL_ECHOLN(y_cos);
  
    delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
    delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi
  
    //SERIAL_ECHOPGM(" delta[X_AXIS]="); SERIAL_ECHO(delta[X_AXIS]);
    //SERIAL_ECHOPGM(" delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
}  

void calculate_delta(float cartesian[3]){
  //reverse kinematics.
  // Perform reversed kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float SCARA_pos[2];
  static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 
  
  SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
  SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.
  
  #if (Linkage_1 == Linkage_2)
    SCARA_C2 = ( ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) ) / (2 * (float)L1_2) ) - 1;
  #else
    SCARA_C2 =   ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2 ) / 45000; 
  #endif
  
  SCARA_S2 = sqrt( 1 - sq(SCARA_C2) );
  
  SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
  SCARA_K2 = Linkage_2 * SCARA_S2;
  
  SCARA_theta = ( atan2(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2(SCARA_K1, SCARA_K2) ) * -1;
  SCARA_psi   =   atan2(SCARA_S2,SCARA_C2);
  
  delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
  delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
  delta[Z_AXIS] = cartesian[Z_AXIS];
  
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);
  
  SERIAL_ECHOPGM("scara x="); SERIAL_ECHO(SCARA_pos[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHOLN(SCARA_pos[Y_AXIS]);
  
  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  
  SERIAL_ECHOPGM("C2="); SERIAL_ECHO(SCARA_C2);
  SERIAL_ECHOPGM(" S2="); SERIAL_ECHO(SCARA_S2);
  SERIAL_ECHOPGM(" Theta="); SERIAL_ECHO(SCARA_theta);
  SERIAL_ECHOPGM(" Psi="); SERIAL_ECHOLN(SCARA_psi);
  SERIAL_ECHOLN(" ");*/
}

#endif

#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static bool red_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
    stat_update += 500; // Update every 0.5s
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
       max_temp = max(max_temp, degHotend(cur_extruder));
       max_temp = max(max_temp, degTargetHotend(cur_extruder));
    }
    #if HAS_TEMP_BED
      max_temp = max(max_temp, degTargetBed());
      max_temp = max(max_temp, degBed());
    #endif
    if((max_temp > 55.0) && (red_led == false)) {
      digitalWrite(STAT_LED_RED, 1);
      digitalWrite(STAT_LED_BLUE, 0);
      red_led = true;
      blue_led = false;
    }
    if((max_temp < 54.0) && (blue_led == false)) {
      digitalWrite(STAT_LED_RED, 0);
      digitalWrite(STAT_LED_BLUE, 1);
      red_led = false;
      blue_led = true;
    }
  }
}
#endif

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {
  
  #if HAS_FILRUNOUT
    if (card.sdprinting && !(READ(FILRUNOUT_PIN) ^ FIL_RUNOUT_INVERTING))
      filrunout();
  #endif

  if (buflen < BUFSIZE - 1) get_command();

  unsigned long ms = millis();

  if (max_inactive_time && ms > previous_millis_cmd + max_inactive_time) kill();

  if (stepper_inactive_time && ms > previous_millis_cmd + stepper_inactive_time
      && !ignore_stepper_queue && !blocks_queued())
    disable_all_steppers();

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ms > chdkHigh + CHDK_DELAY) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL
    
    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
       killCount++;
    else if (killCount > 0)
       killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill();
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enquecommands_P(PSTR("G28"));
        LCD_ALERTMESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif
    
  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #ifdef EXTRUDER_RUNOUT_PREVENT
    if (ms > previous_millis_cmd + EXTRUDER_RUNOUT_SECONDS * 1000)
    if (degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      switch(active_extruder) {
        case 0:
          oldstatus = E0_ENABLE_READ;
          enable_e0();
          break;
        #if EXTRUDERS > 1
          case 1:
            oldstatus = E1_ENABLE_READ;
            enable_e1();
            break;
          #if EXTRUDERS > 2
            case 2:
              oldstatus = E2_ENABLE_READ;
              enable_e2();
              break;
            #if EXTRUDERS > 3
              case 3:
                oldstatus = E3_ENABLE_READ;
                enable_e3();
                break;
            #endif
          #endif
        #endif
      }
      float oldepos = current_position[E_AXIS], oldedes = destination[E_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED / 60. * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS], active_extruder);
      current_position[E_AXIS] = oldepos;
      destination[E_AXIS] = oldedes;
      plan_set_e_position(oldepos);
      previous_millis_cmd = ms; // refresh_cmd_timeout()
      st_synchronize();
      switch(active_extruder) {
        case 0:
          E0_ENABLE_WRITE(oldstatus);
          break;
        #if EXTRUDERS > 1
          case 1:
            E1_ENABLE_WRITE(oldstatus);
            break;
          #if EXTRUDERS > 2
            case 2:
              E2_ENABLE_WRITE(oldstatus);
              break;
            #if EXTRUDERS > 3
              case 3:
                E3_ENABLE_WRITE(oldstatus);
                break;
            #endif
          #endif
        #endif
      }
    }
  #endif

  #ifdef DUAL_X_CARRIAGE
    // handle delayed move timeout
    if (delayed_move_time && ms > delayed_move_time + 1000 && !Stopped) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move();
    }
  #endif

  #ifdef TEMP_STAT_LEDS
    handle_status_leds();
  #endif

  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_all_steppers();

  #if HAS_POWER_SWITCH
    pinMode(PS_ON_PIN, INPUT);
  #endif

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  
  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  for (int i = 5; i--; lcd_update()) delay(200); // Wait a short time
  cli();   // disable interrupts
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

#ifdef FILAMENT_RUNOUT_SENSOR
   void filrunout()
   {
      if filrunoutEnqued == false {
         filrunoutEnqued = true;
         enquecommand("M600");
      }
   }
#endif

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  target_extruder = active_extruder;
  if (code_seen('T')) {
    target_extruder = code_value_short();
    if (target_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(target_extruder);
      return true;
    }
  }
  return false;
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (int i=0; i<EXTRUDERS; i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
