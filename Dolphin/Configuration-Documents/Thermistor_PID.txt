If you are using a preconfigured hotend then you can use one of the value sets below
                                         or
Use the default value (Ultimaker) and run "M303 E0 S200 C8" to auto tune your hotend 
(for multiple extruders replace E0 with E1, E2, etc. and rerun)

Input these values using "M301 Pxx.xx Ixx.xx Dxx.xx" (replace x's with values) and save with M500 
(Note: EEPROM must be active to do this otherwise values must be placed in Configuration.h) 

Ultimaker, Printrbot
  DEFAULT_Kp 22.2
  DEFAULT_Ki 1.08
  DEFAULT_Kd 114

Makergear
  DEFAULT_Kp 7.0
  DEFAULT_Ki 0.1
  DEFAULT_Kd 12

Mendel Parts V9 on 12V
  DEFAULT_Kp 63.0
  DEFAULT_Ki 2.25
  DEFAULT_Kd 440
