/* Menu implementation */
static void lcd_main_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_WATCH, lcd_status_screen);
    if (movesplanned() || IS_SD_PRINTING)
    {
        MENU_ITEM(submenu, MSG_TUNE, lcd_tune_menu);
    }else{
        MENU_ITEM(submenu, MSG_PREPARE, lcd_prepare_menu);
    }
    MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);
#ifdef SDSUPPORT
    if (card.cardOK)
    {
        if (card.isFileOpen())
        {
            if (card.sdprinting)
                MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
            else
                MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
            MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
        }else{
            MENU_ITEM(submenu, MSG_CARD_MENU, lcd_sdcard_menu);
#if SDCARDDETECT < 1
            MENU_ITEM(gcode, MSG_CNG_SDCARD, PSTR("M21"));  // SD-card changed by user
#endif
        }
    }else{
        MENU_ITEM(submenu, MSG_NO_CARD, lcd_sdcard_menu);
#if SDCARDDETECT < 1
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
#endif
    }
#endif
    END_MENU();
}

#ifdef SDSUPPORT
static void lcd_autostart_sd()
{
    card.lastnr=0;
    card.setroot();
    card.checkautostart(true);
}
#endif

void lcd_preheat_pla()
{
    setTargetHotend0(plaPreheatHotendTemp);
    setTargetHotend1(plaPreheatHotendTemp);
    setTargetHotend2(plaPreheatHotendTemp);
    setTargetBed(plaPreheatHPBTemp);
    fanSpeed = plaPreheatFanSpeed;
    lcd_return_to_status();
    setWatch(); // heater sanity check timer
}

void lcd_preheat_abs()
{
    setTargetHotend0(absPreheatHotendTemp);
    setTargetHotend1(absPreheatHotendTemp);
    setTargetHotend2(absPreheatHotendTemp);
    setTargetBed(absPreheatHPBTemp);
    fanSpeed = absPreheatFanSpeed;
    lcd_return_to_status();
    setWatch(); // heater sanity check timer
}

static void lcd_cooldown()
{
    setTargetHotend0(0);
    setTargetHotend1(0);
    setTargetHotend2(0);
    setTargetBed(0);
    lcd_return_to_status();
}

#ifdef BABYSTEPPING
static void lcd_babystep_x()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[X_AXIS]+=(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping X"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}

static void lcd_babystep_y()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[Y_AXIS]+=(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping Y"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}

static void lcd_babystep_z()
{
    if (encoderPosition != 0)
    {
        babystepsTodo[Z_AXIS]+=BABYSTEP_Z_MULTIPLICATOR*(int)encoderPosition;
        encoderPosition=0;
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Babystepping Z"),"");
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_tune_menu;
        encoderPosition = 0;
    }
}
#endif //BABYSTEPPING

static void lcd_tune_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM_EDIT(int3, MSG_SPEED, &feedmultiply, 10, 999);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_1 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_2 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extrudemultiply, 10, 999);

#ifdef BABYSTEPPING
    #ifdef BABYSTEP_XY
      MENU_ITEM(submenu, "Babystep X", lcd_babystep_x);
      MENU_ITEM(submenu, "Babystep Y", lcd_babystep_y);
    #endif //BABYSTEP_XY
    MENU_ITEM(submenu, "Babystep Z", lcd_babystep_z);
#endif
#ifdef FILAMENTCHANGEENABLE
     MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600"));
#endif
    END_MENU();
}

static void lcd_prepare_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
#ifdef SDSUPPORT
    #ifdef MENU_ADDAUTOSTART
      MENU_ITEM(function, MSG_AUTOSTART, lcd_autostart_sd);
    #endif
#endif
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    //MENU_ITEM(gcode, MSG_SET_ORIGIN, PSTR("G92 X0 Y0 Z0"));
    MENU_ITEM(function, MSG_PREHEAT_PLA, lcd_preheat_pla);
    MENU_ITEM(function, MSG_PREHEAT_ABS, lcd_preheat_abs);
    MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);
#if PS_ON_PIN > -1
    if (powersupply)
    {
        MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
    }else{
        MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
    }
#endif
    MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);
    END_MENU();
}

float move_menu_scale;
static void lcd_move_menu_axis();

static void lcd_move_x()
{
    if (encoderPosition != 0)
    {
        current_position[X_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[X_AXIS] < min_pos[0])
            current_position[X_AXIS] = min_pos[0];
        if (max_software_endstops && current_position[X_AXIS] > max_pos[0])
            current_position[X_AXIS] = max_pos[0];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[X_AXIS]/60, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("X"), ftostr31(current_position[X_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_y()
{
    if (encoderPosition != 0)
    {
        current_position[Y_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[Y_AXIS] < min_pos[1])
            current_position[Y_AXIS] = min_pos[1];
        if (max_software_endstops && current_position[Y_AXIS] > max_pos[1])
            current_position[Y_AXIS] = max_pos[1];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[Y_AXIS]/60, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Y"), ftostr31(current_position[Y_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_z()
{
    if (encoderPosition != 0)
    {
        current_position[Z_AXIS] += float((int)encoderPosition) * move_menu_scale;
        if (min_software_endstops && current_position[Z_AXIS] < min_pos[2])
            current_position[Z_AXIS] = min_pos[2];
        if (max_software_endstops && current_position[Z_AXIS] > max_pos[2])
            current_position[Z_AXIS] = max_pos[2];
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[Z_AXIS]/60, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Z"), ftostr31(current_position[Z_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}
static void lcd_move_e()
{
    if (encoderPosition != 0)
    {
        current_position[E_AXIS] += float((int)encoderPosition) * move_menu_scale;
        encoderPosition = 0;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[E_AXIS]/60, active_extruder);
        lcdDrawUpdate = 1;
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Extruder"), ftostr31(current_position[E_AXIS]));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_move_menu_axis;
        encoderPosition = 0;
    }
}

static void lcd_move_menu_axis()
{
    START_MENU();
    MENU_ITEM(back, MSG_MOVE_AXIS, lcd_move_menu);
    MENU_ITEM(submenu, "Move X", lcd_move_x);
    MENU_ITEM(submenu, "Move Y", lcd_move_y);
    if (move_menu_scale < 10.0)
    {
        MENU_ITEM(submenu, "Move Z", lcd_move_z);
        MENU_ITEM(submenu, "Extruder", lcd_move_e);
    }
    END_MENU();
}

static void lcd_move_menu_10mm()
{
    move_menu_scale = 10.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_1mm()
{
    move_menu_scale = 1.0;
    lcd_move_menu_axis();
}
static void lcd_move_menu_01mm()
{
    move_menu_scale = 0.1;
    lcd_move_menu_axis();
}

static void lcd_move_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_PREPARE, lcd_prepare_menu);
    MENU_ITEM(submenu, "Move 10mm", lcd_move_menu_10mm);
    MENU_ITEM(submenu, "Move 1mm", lcd_move_menu_1mm);
    MENU_ITEM(submenu, "Move 0.1mm", lcd_move_menu_01mm);
    //TODO:X,Y,Z,E
    END_MENU();
}

static void lcd_control_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
#ifndef DISABLE_LCD_MOTION_MENU
    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
#endif
#ifdef DOGLCD
//    MENU_ITEM_EDIT(int3, MSG_CONTRAST, &lcd_contrast, 0, 63);
    MENU_ITEM(submenu, MSG_CONTRAST, lcd_set_contrast);
#endif
#ifdef FWRETRACT
    MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
    MENU_ITEM(function, MSG_LOAD_EPROM, Config_RetrieveSettings);
#endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, Config_ResetDefault);
    END_MENU();
}

static void lcd_control_temperature_menu()
{
#ifdef PIDTEMP
    // set up temp variables - undo the default scaling
    raw_Ki = unscalePID_i(Ki);
    raw_Kd = unscalePID_d(Kd);
#endif

    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_1 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE1, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_2 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE2, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15);
#endif
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
#endif
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &fanSpeed, 0, 255);
#ifdef AUTOTEMP
    MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &autotemp_enabled);
    MENU_ITEM_EDIT(float3, MSG_MIN, &autotemp_min, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float3, MSG_MAX, &autotemp_max, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float32, MSG_FACTOR, &autotemp_factor, 0.0, 1.0);
#endif
#ifdef PIDTEMP
    MENU_ITEM_EDIT(float52, MSG_PID_P, &Kp, 1, 9990);
    // i is typically a small value so allows values below 1
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I, &raw_Ki, 0.01, 9990, copy_and_scalePID_i);
    MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D, &raw_Kd, 1, 9990, copy_and_scalePID_d);
# ifdef PID_ADD_EXTRUSION_RATE
    MENU_ITEM_EDIT(float3, MSG_PID_C, &Kc, 1, 9990);
# endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
#ifndef DISABLE_PREHEAT_MENU
    MENU_ITEM(submenu, MSG_PREHEAT_PLA_SETTINGS, lcd_control_temperature_preheat_pla_settings_menu);
    MENU_ITEM(submenu, MSG_PREHEAT_ABS_SETTINGS, lcd_control_temperature_preheat_abs_settings_menu);
#endif
    END_MENU();
}

#ifndef DISABLE_PREHEAT_MENU
static void lcd_control_temperature_preheat_pla_settings_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &plaPreheatFanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &plaPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &plaPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}

static void lcd_control_temperature_preheat_abs_settings_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &absPreheatFanSpeed, 0, 255);
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &absPreheatHotendTemp, 0, HEATER_0_MAXTEMP - 15);
#if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &absPreheatHPBTemp, 0, BED_MAXTEMP - 15);
#endif
#ifdef EEPROM_SETTINGS
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
#endif
    END_MENU();
}
#endif

#ifndef DISABLE_LCD_MOTION_MENU
static void lcd_control_motion_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(float32, MSG_ZPROBE_ZOFFSET, &bed_level_probe_offset[2], 0.5, 50);
    MENU_ITEM_EDIT(float5, MSG_ACC, &acceleration, 500, 99000);
    MENU_ITEM_EDIT(float3, MSG_VXY_JERK, &max_xy_jerk, 1, 990);
    MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &max_z_jerk, 0.1, 990);
    MENU_ITEM_EDIT(float3, MSG_VE_JERK, &max_e_jerk, 1, 990);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &max_feedrate[X_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &max_feedrate[Y_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &max_feedrate[Z_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &max_feedrate[E_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMIN, &minimumfeedrate, 0, 999);
    MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mintravelfeedrate, 0, 999);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &max_acceleration_units_per_sq_second[X_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &max_acceleration_units_per_sq_second[Y_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &max_acceleration_units_per_sq_second[Z_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &max_acceleration_units_per_sq_second[E_AXIS], 100, 99000, reset_acceleration_rates);
    MENU_ITEM_EDIT(float5, MSG_A_RETRACT, &retract_acceleration, 100, 99000);
    MENU_ITEM_EDIT(float52, MSG_XSTEPS, &axis_steps_per_unit[X_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float52, MSG_YSTEPS, &axis_steps_per_unit[Y_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ZSTEPS, &axis_steps_per_unit[Z_AXIS], 5, 9999);
    MENU_ITEM_EDIT(float51, MSG_ESTEPS, &axis_steps_per_unit[E_AXIS], 5, 9999);
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    MENU_ITEM_EDIT(bool, "Endstop abort", &abort_on_endstop_hit);
#endif
    END_MENU();
}
#endif

#ifdef DOGLCD
static void lcd_set_contrast()
{
    if (encoderPosition != 0)
    {
        lcd_contrast -= encoderPosition;
        if (lcd_contrast < 0) lcd_contrast = 0;
        else if (lcd_contrast > 63) lcd_contrast = 63;
        encoderPosition = 0;
        lcdDrawUpdate = 1;
        u8g.setContrast(lcd_contrast);
    }
    if (lcdDrawUpdate)
    {
        lcd_implementation_drawedit(PSTR("Contrast"), itostr2(lcd_contrast));
    }
    if (LCD_CLICKED)
    {
        lcd_quick_feedback();
        currentMenu = lcd_control_menu;
        encoderPosition = 0;
    }
}
#endif

#ifdef FWRETRACT
static void lcd_control_retract_menu()
{
    START_MENU();
    MENU_ITEM(back, MSG_CONTROL, lcd_control_menu);
    MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &autoretract_enabled);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate, 1, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, 0, 100);
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate, 1, 999);
    END_MENU();
}
#endif

#if SDCARDDETECT == -1
static void lcd_sd_refresh()
{
    card.initsd();
    currentMenuViewOffset = 0;
}
#endif
static void lcd_sd_updir()
{
    card.updir();
    currentMenuViewOffset = 0;
}

void lcd_sdcard_menu()
{
    if (lcdDrawUpdate == 0 && LCD_CLICKED == 0)
        return;	// nothing to do (so don't thrash the SD card)
    uint16_t fileCnt = card.getnrfilenames();
    START_MENU();
    MENU_ITEM(back, MSG_MAIN, lcd_main_menu);
    card.getWorkDirName();
    if(card.filename[0]=='/')
    {
#if SDCARDDETECT == -1
        MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
#endif
    }else{
        MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
    }

    for(uint16_t i=0;i<fileCnt;i++)
    {
        if (_menuItemNr == _lineNr)
        {
            #ifndef SDCARD_RATHERRECENTFIRST
              card.getfilename(i);
            #else
              card.getfilename(fileCnt-1-i);
            #endif
            if (card.filenameIsDir)
            {
                MENU_ITEM(sddirectory, MSG_CARD_MENU, card.filename, card.longFilename);
            }else{
                MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename);
            }
        }else{
            MENU_ITEM_DUMMY();
        }
    }
    END_MENU();
}

#define menu_edit_type(_type, _name, _strFunc, scale) \
    void menu_edit_ ## _name () \
    { \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
        } \
    } \
    void menu_edit_callback_ ## _name () \
    { \
        if ((int32_t)encoderPosition < minEditValue) \
            encoderPosition = minEditValue; \
        if ((int32_t)encoderPosition > maxEditValue) \
            encoderPosition = maxEditValue; \
        if (lcdDrawUpdate) \
            lcd_implementation_drawedit(editLabel, _strFunc(((_type)encoderPosition) / scale)); \
        if (LCD_CLICKED) \
        { \
            *((_type*)editValue) = ((_type)encoderPosition) / scale; \
            lcd_quick_feedback(); \
            currentMenu = prevMenu; \
            encoderPosition = prevEncoderPosition; \
            (*callbackFunc)();\
        } \
    } \
    static void menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) \
    { \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
    }\
    static void menu_action_setting_edit_callback_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue, menuFunc_t callback) \
    { \
        prevMenu = currentMenu; \
        prevEncoderPosition = encoderPosition; \
         \
        lcdDrawUpdate = 2; \
        currentMenu = menu_edit_callback_ ## _name; \
         \
        editLabel = pstr; \
        editValue = ptr; \
        minEditValue = minValue * scale; \
        maxEditValue = maxValue * scale; \
        encoderPosition = (*ptr) * scale; \
        callbackFunc = callback;\
    }
menu_edit_type(int, int3, itostr3, 1)
menu_edit_type(float, float3, ftostr3, 1)
menu_edit_type(float, float32, ftostr32, 100)
menu_edit_type(float, float5, ftostr5, 0.01)
menu_edit_type(float, float51, ftostr51, 10)
menu_edit_type(float, float52, ftostr52, 100)
menu_edit_type(unsigned long, long5, ftostr5, 0.01)

#ifdef REPRAPWORLD_KEYPAD
	static void reprapworld_keypad_move_z_up() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_z();
  }
	static void reprapworld_keypad_move_z_down() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_z();
  }
	static void reprapworld_keypad_move_x_left() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_x();
  }
	static void reprapworld_keypad_move_x_right() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_x();
	}
	static void reprapworld_keypad_move_y_down() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
		lcd_move_y();
	}
	static void reprapworld_keypad_move_y_up() {
		encoderPosition = -1;
		move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_y();
	}
	static void reprapworld_keypad_move_home() {
		enquecommand_P((PSTR("G28"))); // move all axis home
	}
#endif

/** End of menus **/
