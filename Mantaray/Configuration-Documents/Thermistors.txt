--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table (default is 1)

Temperature sensor settings:
-2  = thermocouple with MAX6675 (only for sensor 0)
-1  = thermocouple with AD595
 0  = is not used
 1  = 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 2  = 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 3  = mendel-parts thermistor (4.7k pullup)
 4  = 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
 5  = 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
 6  = 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 7  = 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 71 = 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 8  = 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 9  = 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 10 = 100k RS thermistor 198-961 (4.7k pullup)
 60 = 100k Maker's Tool Works Kapton Bed Thermister
 51 = 100k thermistor - EPCOS (1k pullup)
 52 = 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 55 = 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
