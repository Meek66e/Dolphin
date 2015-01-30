/****************************************************************************************
  Printrboard rev F pin assignments (AT90USB1286)
****************************************************************************************/

#define AT90USB 1286  // Disable MantaraySerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

//Disable JTAG pins so they can be used for the Extrudrboard
#define DISABLE_JTAG       true

#define X_STEP_PIN          0
#define X_DIR_PIN           1
#define X_ENABLE_PIN       39

#define Y_STEP_PIN          2
#define Y_DIR_PIN           3
#define Y_ENABLE_PIN       38

#define Z_STEP_PIN          4
#define Z_DIR_PIN           5
#define Z_ENABLE_PIN       23

#define E0_STEP_PIN         6
#define E0_DIR_PIN          7
#define E0_ENABLE_PIN      19

#define E1_STEP_PIN        24
#define E1_DIR_PIN         25
#define E1_ENABLE_PIN      44

#define E2_STEP_PIN        26
#define E2_DIR_PIN         27
#define E2_ENABLE_PIN      45

#define HEATER_0_PIN       21  // Extruder
#define HEATER_1_PIN       46
#define HEATER_2_PIN       47
#define HEATER_BED_PIN     20  // Bed

// If soft or fast PWM is off then use Teensyduino pin numbering, Mantaray
// fastio pin numbering otherwise
#ifdef FAN_SOFT_PWM || FAST_PWM_FAN
	#define FAN_PIN        22  // Fan
#else
	#define FAN_PIN        16  // Fan
#endif

  #define X_STOP_PIN         35
  #define Y_STOP_PIN         12
  #define Z_STOP_PIN         36
  #define TEMP_0_PIN          1  // Extruder / Analog pin numbering
  #define TEMP_BED_PIN        0  // Bed / Analog pin numbering
  #define SDSS               20

#define TEMP_1_PIN         2
#define TEMP_2_PIN         3

#define SDPOWER            -1
#define LED_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1
#define ALARM_PIN          -1

#define DAC_STEPPER_CURRENT
#define DAC_STEPPER_ADDRESS	0
#define DAC_STEPPER_ORDER 	{3,2,1,0}
#define DAC_STEPPER_MAX 	3520
#define DAC_STEPPER_VREF 	1 //internal Vref, gain 1x = 2.048V
#define DAC_STEPPER_GAIN	0

#ifdef ULTRA_LCD
  #define BEEPER -1

  #define LCD_PINS_RS 9
  #define LCD_PINS_ENABLE 8
  #define LCD_PINS_D4 7
  #define LCD_PINS_D5 6
  #define LCD_PINS_D6 5
  #define LCD_PINS_D7 4

  #define BTN_EN1   16
  #define BTN_EN2   17
  #define BTN_ENC   18//the click

  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0

  #define SDCARDDETECT -1

  //encoder rotation values
  #define encrot0 0
  #define encrot1 2
  #define encrot2 3
  #define encrot3 1
#endif

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN           9
  #define MISO_PIN         11
  #define MOSI_PIN         10
#endif
