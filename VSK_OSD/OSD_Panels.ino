/******* STARTUP PANEL *******/
int AH_COLS = 12;
int AH_ROWS = 7;
int8_t old_col = 0;
int8_t old_line = 0;
int8_t name[6];

void startPanels(){
  panLogo(); // Display our logo  
  do_converts(); // load the unit conversion preferences
}

//------------------ Panel: Startup ArduCam OSD LOGO -------------------------------

void panLogo(){
    osd.setPanel(9, 5);
    osd.openPanel();
    osd.printf_P(PSTR("VSkyline OSD|  ver 1.0"));
    osd.closePanel();
}


/******* PANELS - POSITION *******/

void writePanels(){ 
//if(millis() < (lastMAVBeat + 2200))
//  waitingMAVBeats = 1;
//if(ISd(panel,Warn_BIT)) panWarn(panWarn_XY[0][panel], panWarn_XY[1][panel]); // this must be here so warnings are always checked
  timers();
  //Base panel selection
  //No mavlink data available panel
  if(millis() > (lastMAVBeat + 2200)){
    if (currentBasePanel != 2){
      osd.clear();
      currentBasePanel = 2;
    }   
    //panLogo();
    //waitingMAVBeats = 1;
    //Display our logo and wait... 
    panWaitMAVBeats(10,8); //Waiting for MAVBeats...
  }
  //Flight summary panel
  //Only show flight summary 10 seconds after landing and if throttle < 15
  else if (!motor_armed && (((millis() / 10000) % 2) == 0) && (tdistance > 50)){ 
    if (currentBasePanel != 1){
      osd.clear();
      currentBasePanel = 1;
    }
    panFdata(); 
  }
  //Normal osd panel
  else{
    if ((osd_clear == 1) || (currentBasePanel != 0) || (old_panel != panel) || (old_subpage != subpage)){
      osd.clear();
      osd_clear = 0;
      currentBasePanel = 0;
      old_panel = panel;
      old_subpage = subpage;
//      Serial.print("clear osd ");
    }
    //mainpage
    if(panel == 0){
      //line 1
      panBatteryPercent(1, 0 +vmode_line);//Battery Percent to progress bar
      panBatt_A(8, 0+vmode_line);//Battery Voltage in Volt
      panCur_A(15, 0+vmode_line);//Battery Current in Ampe
      panRSSI(23, 0+vmode_line);//RSSI
      //line 2
      panBatteryCapacity(11,12+vmode_line);
      panPitch(9, 1+vmode_line);
      panRoll(18, 1+vmode_line);
      panYaw(13, 1+vmode_line);
      panTime(1, 12+vmode_line);//time
      //central and bottom
//      AH_ROWS = 7 + vmode_line*2;
      AH_ROWS = 7;
      panHorizon(8, 3+ vmode_line);//Horizon
      panCentral(14, 6+ vmode_line);//Central point
//      panAlt(23, 6+ vmode_line);//altitude
//      panVel(1, 6+ vmode_line);//velocity
      //panGPS(1, 11);//GPS
//      panGPSats(1, 12+ vmode_line*2);      
      panLowBattery(7, 11+ vmode_line);//warning Low Battery
//      panFlightMode(23, 12+ vmode_line*2);//mode
      panHighPitch(8, 10+ vmode_line);
      panLowPitch(8, 2 +vmode_line);
      panPilotName(23, 12);
    }
  //Lite1 page
    else if(panel == 1){
      panBatteryPercent(1, 0);//Battery Percent to progress bar
      panBatteryCapacity(1, 1);
      panBatt_A(8, 0);//Battery Voltage in Volt
      panCur_A(15, 0);//Battery Current in Ampe
      panRSSI(23, 0);//RSSI
      panTime(23, 1);//time done
      panLowBattery(10, 12);
      AH_ROWS = 11;
      panHorizon(8, 1);//Horizon
      panCentral(14, 6);
    }
  //Lite2page
    else if(panel == 2){
      panBatteryPercent(1, 0);
      panBatteryCapacity(1, 1);
      panBatt_A(8, 0);//Battery Voltage in Volt
      panCur_A(15, 0);//Battery Current in Ampe
      panRSSI(23, 0);//RSSI
      panTime(23, 1);//time
      panLowBattery(9, 3);
    }
  //PID page
    else if(panel == 3){
      panBatteryPercent(1, 0);
      panBatt_A(8, 0);//Battery Voltage in Volt
      panCur_A(15, 0);//Battery Current in Ampe
      panRSSI(23, 0);//RSSI
      panTime(23, 1);//time
      panLowBattery(10, 12);
      AH_ROWS = 11;
      panHorizon(8, 1);//Horizon
      panCentral(14, 6);
      panPID(1, 1);
    }
    //Setting menu page
    else if(panel == 4){
      //setting first page
      if(subpage == 0){
        panBatteryPercent(1, 0);
        panBatt_A(8, 0);
        panCur_A(15, 0);
        panRSSI(23, 0);
        panSbus();
        panPilotName(16, 9);
        panSettingMenu();
        //panCursor1();
      }
      else if(subpage == 1){
        panRCsetupTop();
        panCursor2();
//        panProgressBar(17, 8, chan1_raw, 2510, 892);
//        panProgressBar(17, 9, chan3_raw, 1848, 210);
//        panProgressBar(17, 10, chan2_raw, 1835, 210);
//        panProgressBar(17, 11, chan4_raw, 1848, 210);
      }
      else if(subpage == 2){
        panVideoSetup();
        panCursor2();
      }
      else if(subpage == 3){
        panIMUsetup();
        panCursor2();
      }
      else if(subpage == 4){
        panMotor();
        panCursor2();
      }
      else if(subpage == 5){
        panPID(5, 3);
        panCursor3();
      }
      //Radio calibration
      else if(subpage == 7){
        panRadioCal();
        panProgressBar(13, 5, chan1_raw, 2524, 1024);
        panProgressBar(13, 6, chan4_raw, 1774, 274);
        panProgressBar(13, 7, chan2_raw, 1774, 274);
        panProgressBar(13, 8, chan3_raw, 1774, 274);
      }
      //RC setup
      else if(subpage == 8){
        panRadioStatus();
        panProgressBar(17, 3, chan1_raw, 2524, 1024);
        panProgressBar(17, 4, chan3_raw, 1774, 274);
        panProgressBar(17, 5, chan2_raw, 1774, 274);
        panProgressBar(17, 6, chan4_raw, 1774, 274);
      }
      else if(subpage == 9){
        panBatteryPercent(1, 0);
        panBatt_A(8, 0);
        panCur_A(15, 0);
        panRSSI(23, 0);
        panPilotName(16, 9);
        panSettingMenu();
        osd.setPanel(1, 1);
        osd.openPanel();
        osd.printf_P(PSTR("if you want calib your radio|please center all stick     "));
        osd.closePanel();
      }
    }
  }
}

/******* PANELS - DEFINITION *******/
/* **************************************************************** */

/* **************************************************************** */
// Panel  : COG Course Over Ground
// Needs  : X, Y locations
// Output : 
// Size   : 
// Staus  : done

void panProgressBar(int first_col, int first_line, int value, int max_value, int min_value){
    int resolution = (max_value - min_value)/24;
    int temp_value = value;
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("%c"), 0x88);
    for(int i = 0; i < 6; i++){
       if(temp_value >= (4*resolution + min_value)){
         osd.printf_P(PSTR("%c"), 0x89);
       } else if(temp_value >= (3*resolution + min_value)){
         osd.printf_P(PSTR("%c"), 0x8A);
       } else if(temp_value >= (2*resolution + min_value)){
         osd.printf_P(PSTR("%c"), 0x8B);
       } else if(temp_value >= (resolution + min_value)){
         osd.printf_P(PSTR("%c"), 0x8C);
       } else osd.printf_P(PSTR("%c"),0x8D);
       temp_value =  temp_value - (4*resolution);
    }
    osd.printf_P(PSTR("%c"), 0x8E);
    osd.closePanel();
}

void panSbus(){
   osd.setPanel(1, 1);
   osd.openPanel();
   if(rssi2 == 0){
      osd.printf_P(PSTR("radio not detect            |                       "));
      osd.closePanel();
   } else {
      osd.printf_P(PSTR("if you want calib your radio|please center all stick"));
      osd.closePanel();
      panCursor1();
   }   
}

void panPilotName(int first_col, int first_line){
       //name processing
    name[0] = 0x1F& ((uint8_t) name32);
    name[1] = 0x1F& ((uint8_t) (name32 >> 5));
    name[2] = 0x1F& ((uint8_t) (name32 >> 10));
    name[3] = 0x1F& ((uint8_t) (name32 >> 15));
    name[4] = 0x1F& ((uint8_t) (name32 >> 20));
    name[5] = 0x1F& ((uint8_t) (name32 >> 25));
//    Serial.printf(" Processing %i %i %i %i %i %i", name[0], name[1], name[2], name[3], name[4], name[5]);
    for (int i = 0; i < 6; i++){
       if(name[i] == 0){
          name[i] = 0x20;
       } else if(name[i] < 27){
          name[i] += 0x60;
       } else {
          name[i] += 21;
       }       
    }
    osd.setPanel(first_col, first_line + vmode_line);
    osd.openPanel();
//    if(blinker && ((page_id & 0x0f00) == 0x0700)){
//      osd.printf("%c%c%c%c%c%c", 0x20, 0x20, 0x20, 0x20, 0x20, 0x20);      
//    }
//    else {
//      osd.printf("%c%c%c%c%c%c", name[0], name[1], name[2], name[3], name[4], name[5]);
//    }
   if((page_id & 0x0f00) == 0x0700){
     for(int i = 1; i < 7; i++){
        if((page_id & 0x000f) == i){
           if(blinker){
             if(name[i-1] == 0x20){
               osd.printf("%c", 0x5F);
             }else {
               osd.printf("%c", 0x20);
             }
           } else {
             osd.printf("%c", name[i-1]);
           }
        } else 
        osd.printf("%c", name[i-1]);
     }

   }
   else osd.printf("%c%c%c%c%c%c", name[0], name[1], name[2], name[3], name[4], name[5]);
   osd.closePanel(); 
}

void panRadioCal(){
   int old_p_counter, p_counter;
   p_counter = page_id & 0x000f;
   osd.setPanel(3, 0);
   osd.openPanel();
   osd.printf_P(PSTR("radio calibration||||%4.0i"), rcerrors);
//   osd.printf_P(PSTR("  move all center"));
   osd.closePanel();
   
   osd.setPanel(5, 1);
   osd.openPanel();
   if(old_p_counter != p_counter){
      osd.printf_P(PSTR("                        |                        |                         ")); 
   }
   osd.closePanel();
   
   osd.setPanel(5, 1);
   osd.openPanel();
   if(p_counter == 0){
       osd.printf_P(PSTR("move all center"));
   } else if(p_counter == 1){
       osd.printf_P(PSTR("move throttle to minimum|      and hold"));
   } else if(p_counter == 2){
       osd.printf_P(PSTR("move throttle center|& yaw left %c and hold|||thrott"), 0xA4);
   } else if(p_counter == 3){
       osd.printf_P(PSTR("move roll left %c and hold|||||yaw"), 0xA4);
   } else if(p_counter == 4){
       osd.printf_P(PSTR("move pitch up %c and hold||||||roll"), 0xA6);
   } else if(p_counter == 5){
       osd.printf_P(PSTR("calib complete|please roll right %c to save|roll left %c to quit|||||pitch"), 0xA5, 0xA4);
   }
   osd.closePanel();
   old_p_counter = p_counter;
}

void panRadioStatus(){
    osd.setPanel(3, 0);
    osd.openPanel();
    osd.printf_P(PSTR("radio status"));
    osd.printf_P(PSTR("|radio type    :sbus"));
    osd.printf_P(PSTR("|radio quanlity:%2.0i"), rssi2);
    osd.printf("||thrott   %4.0i", chan1_raw-1024);
    osd.printf("|pitch    %4.0i", chan3_raw-1024);
    osd.printf("|roll     %4.0i", chan2_raw-1024);
    osd.printf("|yaw      %4.0i", chan4_raw-1024);
    osd.printf("|osd page     ");
    osd.printf("|osd value     ");
    osd.closePanel();
    
    osd.setPanel(4, 11 + 2*vmode_line);
    osd.openPanel();
    osd.printf_P(PSTR("move pitch %c & roll %c|  go back radio page"), 0xA7, 0xA4);
    osd.closePanel();
}
void panLowBattery(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if(osd_vbat_A < batt_warn_level){
       osd.printf_P(PSTR("LOW BATTERY %5.2f"), (double) osd_vbat_A); 
    }
    else osd.printf_P(PSTR("                     "));
    osd.closePanel();
}

void panCentral(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
//    osd.printf("\xAD\xAE|\xAF\xB5");
    osd.printf_P(PSTR("\x90\x91"));
    osd.closePanel(); 
}

void panLowPitch(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if(osd_pitch < -44){
      osd.printf_P(PSTR("LOW PITCH "));
      osd.printf("%3.0i", osd_pitch);     
    } else {
        osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
    }
    osd.closePanel();
}

void panHighPitch(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if(osd_pitch > 43){
      osd.printf_P(PSTR("HIGH PITCH "));
      osd.printf("%3.0i", osd_pitch);      
   } else {
        osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
   }
   osd.closePanel();
}

void panYaw(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3i", 0xAF, osd_yaw);
    osd.closePanel();
}

void panPID(int first_col, int first_line){
    osd.setPanel(2, 0);
    osd.openPanel();
    osd.printf_P(PSTR("pid setup"));
    osd.closePanel();
    osd.setPanel(4, 1);
    osd.openPanel();
    osd.printf_P(PSTR("load profile: %1.0f"), profile);
    osd.printf_P(PSTR("|pid type    : rewrite"));
    osd.closePanel();
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("%cp:  %4.1f"), 0x07, p_pitch);
    osd.printf_P(PSTR("| i: %4.3f"), i_pitch);
    osd.printf_P(PSTR("| d: %5.0f"), d_pitch);
    osd.printf_P(PSTR("| r: %5.2f"), rate_pitch);
    osd.printf_P(PSTR("|%cp:  %4.1f"), 0x06, p_roll);
    osd.printf_P(PSTR("| i: %4.3f"), i_roll);
    osd.printf_P(PSTR("| d: %5.0f"), d_roll);
    osd.printf_P(PSTR("| r: %5.2f"), rate_roll);
//    osd.printf_P(PSTR("\x07\x70%2.1f|\x20\x69|\x20\x64|\x20\x72"), p_pitch);
//    osd.printf_P(PSTR("|\x06\x70|\x20\x69|\x20\x64|\x20\x72"));
    osd.closePanel();
    
    osd.setPanel(first_col + 12, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("%cp:  %4.1f"), 0xAF, p_yaw);
    osd.printf_P(PSTR("| i: %4.3f"), i_yaw);
    osd.printf_P(PSTR("| d: %5.0f"), d_yaw);
    osd.printf_P(PSTR("| r: %5.2f"), rate_yaw);
    osd.printf_P(PSTR("||save & exit"));
//    osd.printf_P(PSTR("\xAF\x70|\x20\x69|\x20\x64|\x20\x67"));
    osd.closePanel();
}

void panSettingMenu(){
    osd.setPanel(8, 3 + vmode_line);
    osd.openPanel();
    osd.printf_P(PSTR("vsk osd menu"));
    //osd.printf("|%4x", page_id);
    osd.closePanel();
    osd.setPanel(4, 4 + vmode_line);
    osd.openPanel();
    osd.printf_P(PSTR("rc setup|pid setup|imu setup|motor & esc setup|video & video tx setup|pilot name:"));
    osd.closePanel();
    osd.setPanel(3, 10 + vmode_line);
    osd.openPanel();
    osd.printf_P(PSTR("move throttle %c & yaw %c|   go to flying page"), 0xA7, 0xA4);
    osd.closePanel();
}

void panCursor1(){
    if(blinker){
      osd.setPanel(pos_col, pos_line + vmode_line);
      osd.openPanel();
      osd.printf_P(PSTR("%c"), 0xA8);
      osd.closePanel();
    } else {
      osd.setPanel(pos_col, pos_line +vmode_line);
      osd.openPanel();
      osd.printf_P(PSTR(" "));
      osd.closePanel();
    }
    if ((old_col != pos_col) || (old_line != pos_line)){
      osd.setPanel(old_col, old_line+ vmode_line);
      osd.openPanel();
      osd.printf_P(PSTR(" "));
      osd.closePanel();
      old_col = pos_col;
      old_line = pos_line;
    }
}

void panCursor2(){
    if(blinker){
      if(pos_line == 0){
        pos_col = 1;
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA9);
        osd.closePanel();
      }
      else {
        pos_col = 2;
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA8);
        osd.closePanel();
      }
    } else {
      osd.setPanel(pos_col, pos_line);
      osd.openPanel();
      osd.printf_P(PSTR(" "));
      osd.closePanel();
    }
    if ((old_col != pos_col) || (old_line != pos_line)){
      osd.setPanel(old_col, old_line);
      osd.openPanel();
      osd.printf_P(PSTR(" "));
      osd.closePanel();
      old_col = pos_col;
      old_line = pos_line;
    }
}

void panCursor3(){
    if(blinker){
      if(pos_line == 0){
        pos_col = 1;
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA9);
        osd.closePanel();
      }else if(pos_line < 3){
        pos_col = 2;
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA8);
        osd.closePanel();
      }else if(pos_line < 11){
        pos_col = 4;
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA8);
        osd.closePanel(); 
      }else {
        if(pos_line == 15)
          pos_line = pos_line + 1;
        pos_col = 15;
        osd.setPanel(pos_col, pos_line-8);
        osd.openPanel();
        osd.printf_P(PSTR("%c"), 0xA8);
        osd.closePanel();       
      }
    } else {
      if(pos_line < 11){        
        osd.setPanel(pos_col, pos_line);
        osd.openPanel();
        osd.printf_P(PSTR(" "));
        osd.closePanel();
      }else {
        if(pos_line == 15)
          pos_line = pos_line + 1;
        osd.setPanel(pos_col, pos_line-8);
        osd.openPanel();
        osd.printf_P(PSTR(" "));
        osd.closePanel();
      }
    }
    if ((old_col != pos_col) || (old_line != pos_line)){
      if(old_line > 10){
        old_line = old_line-8;
      }
      osd.setPanel(old_col, old_line);
      osd.openPanel();
      osd.printf_P(PSTR(" "));
      osd.closePanel();
      old_col = pos_col;
      old_line = pos_line;
    }
}

void panMotor(){
    osd.setPanel(2, 0);
    osd.openPanel();
    osd.printf_P(PSTR("motor&esc setup"));
    osd.closePanel();
    
    osd.setPanel(3, 1);
    osd.openPanel();
    osd.printf_P(PSTR("esc calibration <<"));
    osd.printf_P(PSTR("||motor a dir: %s <<"), ndir);
    osd.printf_P(PSTR("|motor b dir: %s <<"), rdir);
    osd.printf_P(PSTR("|motor c dir: %s <<"), ndir);
    osd.printf_P(PSTR("|motor d dir: %s <<"), rdir);
    osd.closePanel();
    
    osd.setPanel(12, 8);
    osd.openPanel();
    osd.printf("\xA0\x61  b\xA2|  \x9D\x9E|\xA1\x64  c\xA3");
    osd.closePanel();
}

void panRCsetupTop(){
    osd.setPanel(2,0);
    osd.openPanel();
    osd.printf_P(PSTR("rc setup"));
    osd.closePanel();
    
    osd.setPanel(3, 1);
    osd.openPanel();
    osd.printf("radio name:%s <<", rctype);
    osd.printf("|radio status check >>");
    osd.printf("|radio calibration >>");
    osd.printf("|throttle deadband:%3.0i%c <<", thr_dband, 0x15);
    osd.printf("|p/r      deadband:%3.0i%c <<", thr_dband, 0x15);
    osd.printf("|p/r expo         :%3i%c <<", pr_expo, 0x25);
    osd.printf("|throttle expo    :%3i%c <<", thr_expo, 0x25);
//    osd.printf("||thrott   %4.0i", chan1_raw);
//    osd.printf("|pitch    %4.0i", chan3_raw);
//    osd.printf("|roll     %4.0i", chan2_raw);
//    osd.printf("|yaw      %4.0i", chan4_raw);
//    osd.printf("|flight   ");
//    osd.printf("|home     ");
    osd.closePanel();
}

void panVideoSetup(){
    osd.setPanel(2, 0);
    osd.openPanel();
    osd.printf_P(PSTR("video & video tx setup"));
    osd.closePanel();
    osd.setPanel(3, 1);
    osd.openPanel();
    if(video_mode_old == 1)
      osd.printf("video mode: pal ");
    else {
      osd.printf("video mode: ntsc");
    }
    osd.printf("|video tx channel: 1");
    osd.printf("|video tx band   : a");
    osd.printf("|osd brightness: low");
    osd.printf("|    save & exit    ");
    osd.printf("||||video frequency: 5740mhz");
    osd.closePanel();
}

void panIMUsetup(){
    osd.setPanel(2, 0);
    osd.openPanel();
    osd.printf_P(PSTR("imu setup"));
    osd.closePanel();
    osd.setPanel(3, 1);
    osd.openPanel();
    osd.printf_P(PSTR("imu level <<|"));
    osd.printf_P(PSTR("|imu rotation: <<"));
    osd.printf_P(PSTR("|imu gyro status:good"));
    osd.printf_P(PSTR("|imu acc  status:good"));
    osd.printf_P(PSTR("|imu mag  status:no connect"));
    osd.printf_P(PSTR("|imu baro status:no connect"));
    osd.printf_P(PSTR("|gps      status:no connect"));
    osd.printf_P(PSTR("||imu pitch: %4.0i"), osd_pitch);
    osd.closePanel();
}

//void panCOG(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    
//    osd_COG_arrow_rotate_int = round(((osd_cog / 100) - osd_heading)/360.0 * 16.0 +1); //Convert to int 1-16 
//    if(osd_COG_arrow_rotate_int < 0 ) osd_COG_arrow_rotate_int += 16;
//    if(osd_COG_arrow_rotate_int == 0) osd_COG_arrow_rotate_int = 16;    
//    if(osd_COG_arrow_rotate_int == 17) osd_COG_arrow_rotate_int = 1;
//    
//    if (((osd_cog / 100) - osd_heading) > 180){
//       off_course = (osd_cog / 100 - osd_heading) - 360;
//    }else if (((osd_cog / 100) - osd_heading) < -180){
//       off_course = (osd_cog / 100 - osd_heading) + 360;
//    }else{
//       off_course = (osd_cog / 100 - osd_heading);
//    }
//    
//    showArrow((uint8_t)osd_COG_arrow_rotate_int,2);
//    osd.closePanel();
//}

// Panel  : ODO
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

//void panDistance(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    if ((tdistance * converth) > 9999.0) {
//      osd.printf("%c%5.2f%c", 0x8f, ((tdistance * converth) / distconv), distchar);
//    }else{
//      osd.printf("%c%5.0f%c", 0x8f, (tdistance * converth), high);
//    }
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panFdata
// Needs  : X, Y locations
// Output : 
// Size   : 
// Staus  : done
void panFdata()
{
  osd.setPanel(11, 4);
  osd.openPanel();
  osd.printf("%c%3i%c%02i|%c%5i%c|%c%5i%c|%c%5i%c|%c%5i%c|%c%10.6f|%c%10.6f", 0x08,((int)total_flight_time_seconds/60)%60,0x3A,(int)total_flight_time_seconds%60, 0x0B, (int)((max_home_distance) * converth), high, 0x8F, (int)((tdistance) * converth), high,0x14,(int)(max_osd_groundspeed * converts),spe,0x12, (int)(max_osd_home_alt * converth), high, 0x03, (double)osd_lat, 0x04, (double)osd_lon);
  osd.closePanel();
}

/* **************************************************************** */
// Panel  : pantemp
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

//void panTemp(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    //do_converts();
//    osd.printf("%5.1f%c", (float(temperature / 10 * tempconv + tempconvAdd) / 100), temps);    
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : efficiency
// Needs  : X, Y locations
// Output : 
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done
//void panEff(int first_col, int first_line)
//{
//  osd.setPanel(first_col, first_line);
//  osd.openPanel();
//  //Check takeoff just to prevent inicial false readings
//  if (motor_armed)
//  {
//    if(osd_battery_remaining_A != last_battery_reading)
//    {
//      remaining_estimated_flight_time_seconds = ((float)osd_battery_remaining_A * total_flight_time_milis / (max_battery_reading - osd_battery_remaining_A)) / 1000;
//      last_battery_reading = osd_battery_remaining_A;
//    }
//    osd.printf("%c%2i%c%02i", 0x17,((int)remaining_estimated_flight_time_seconds/60)%60,0x3A,(int)remaining_estimated_flight_time_seconds%60);
//  }
//  osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panCh
// Needs  : X, Y locations
// Output : Scaled channel values from MAVLink
// Size   
// Staus  : done

//void panCh(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
    
//    osd.printf("%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i|%c%c%5i", 0x43, 0x31, chan1_raw, 0x43, 0x32, chan2_raw, 0x43, 0x33, chan3_raw, 0x43, 0x34, chan4_raw, 0x43, 0x35, chan5_raw, 0x43, 0x36, chan6_raw); 
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panRSSI(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
//    if((rssiraw_on % 2 == 0))
//    {
//       if(osd_rssi < rssipersent) osd_rssi = rssipersent;
//       if(osd_rssi > rssical) osd_rssi = rssical;
////       if(rssiraw_on == 0) 
//       rssi = (int16_t)((float)((int16_t)osd_rssi - rssipersent)/(float)(rssical-rssipersent)*100.0f);
//       if(rssiraw_on == 8) rssi = (int16_t)((float)(chan8_raw / 10 - rssipersent)/(float)(rssical-rssipersent)*100.0f);
//    }
//    if(rssiraw_on == 1) rssi = (int16_t)osd_rssi;
//    if(rssiraw_on == 9) rssi = chan8_raw;

//    if(rssi > 100.0) rssi = 100;

    osd.printf("%c%3i%c", 0x09, rssi2, 0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panCur_A(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%5.2f%c", (float(osd_curr_A) * 0.01), 0x0e);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

void panAlt(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //if(EEPROM.read(SIGN_MSL_ON_ADDR) != 0) osd.printf_P("\x11");
    if(EEPROM.read(SIGN_MSL_ON_ADDR) != 0) osd.printf("%c", 0x11);
    osd.printf("%5.0f%c", (double)(osd_alt_gps * converth), high);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Alt symbol and altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

//void panClimb(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf("%c%4.0f%c%c", 0x15, int(vs / 10.0) * 10.0, climbchar, 0x20);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Alt symbol and home altitude value in meters from MAVLink
// Size   : 1 x 7Hea  (rows x chars)
// Staus  : done

//void panHomeAlt(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    //if(EEPROM.read(SIGN_HA_ON_ADDR) != 0) osd.printf_P('\x12');
//    if(EEPROM.read(SIGN_HA_ON_ADDR) != 0) osd.printf("%c", 0x12);
//    osd.printf("%5.0f%c", (double)(osd_alt_rel * converth), high);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panVel(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //if(EEPROM.read(SIGN_GS_ON_ADDR) != 0) osd.printf_P("\x14");
    //if(EEPROM.read(SIGN_GS_ON_ADDR) != 0) osd.printf("%c", 0x14);
    osd.printf("%c%3.0f%c",0xB3 ,(double)(osd_groundspeed * converts),spe);
    if(osd_groundspeed > osd_maxspeed){
      osd_maxspeed = osd_groundspeed;
    }
    osd.printf("%c%c%3.0f%c", '|',0xB4, (double)(osd_maxspeed * converts),spe);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panAirSpeed
// Needs  : X, Y locations
// Output : Airspeed value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

//void panAirSpeed(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    //if(EEPROM.read(SIGN_AS_ON_ADDR) != 0) osd.printf_P("\x13");
//    if(EEPROM.read(SIGN_AS_ON_ADDR) != 0) osd.printf("%c", 0x13);
//    osd.printf("%3.0f%c", (double)(osd_airspeed * converts), spe); 
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle value from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panThr(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%3.0i%c",osd_throttle,0x25);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery state from MAVlink with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panBatteryPercent(int first_col, int first_line){
    int batt_temp = osd_battery_remaining_A;
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("\x88");
    for(int i = 0; i < 5; i++){
       if(batt_temp > 20){
          osd.printf("\x89"); 
       } else if(batt_temp > 15){
             osd.printf("\x8A");
       } else if(batt_temp > 10){
              osd.printf("\x8B"); 
       } else if(batt_temp > 5){
              osd.printf("\x8C"); 
       } else osd.printf("\x8D");
       batt_temp -=20;
    }
    osd.printf("%c", '\x8E');
    osd.closePanel();
}

void panBatteryCapacity(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c", 0x17, mah_used, 0x01);
    //osd.printf("|%16.0i", lastMAVBeat);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : 
// Needs  : X, Y locations
// Output : Time from start with symbols
// Size   : 1 x 7  (rows x chars)
// Staus  : done

void panTime(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%2i%c%02i",0xB2, ((int)total_flight_time_seconds/60)%60,0x3A,(int)total_flight_time_seconds%60);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Home Symbol with distance to home in meters
// Size   : 1 x 7  (rows x chars)
// Staus  : done

//void panHomeDis(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf("%c%5.0f%c", 0x0b, (double)((osd_home_distance) * converth), high);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panCenter
// Needs  : X, Y locations
// Output : 2 row croshair symbol created by 2 x 4 chars
// Size   : 2 x 4  (rows x chars)
// Staus  : done

//void panCenter(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    osd.printf_P(PSTR("\x05\x03\x04\x05|\x15\x13\x14\x15"));
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : 12 x 4 Horizon line surrounded by 2 cols (left/right rules)
// Size   : 14 x 4  (rows x chars) 5x15
// Staus  : done

void panHorizon(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
  
//    osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\xC6\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xC5\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
    if(AH_ROWS == 11){      
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\xC6\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xC5|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
//    } 
//    else if (AH_ROWS == 9){
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\xC6\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xC5|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
//      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));      
    } else {
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\xC6\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xC5|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20|"));
      osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"));
      }
    osd.closePanel();
    showHorizon((first_col + 1), first_line);
    //Show ground level on  HUD
    //showILS(first_col, first_line);
}

/* **************************************************************** */
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panPitch(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3i", 0x07, osd_pitch);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
// Size   : 1 x 6  (rows x chars)
// Staus  : done

void panRoll(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%4i", 0x06, osd_roll);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
// Size   : 1 x 8  (rows x chars)
// Staus  : done

void panBatt_A(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    /*************** This commented code is for the next ArduPlane Version
    if(osd_battery_remaining_A > 100){
        osd.printf(" %c%5.2f%c", 0xbc, (double)osd_vbat_A, 0x0d);
    else osd.printf("%c%5.2f%c%c", 0xbc, (double)osd_vbat_A, 0x0d, osd_battery_pic_A);
    */
//    osd.printf("%c%5.2f%c", 0xbc, (double)osd_vbat_A, 0x0d);
    osd.printf("%5.2f%c", (double)osd_vbat_A, 0x0d);
    osd.closePanel();
}

//------------------ Panel: Waiting for MAVLink HeartBeats -------------------------------

void panWaitMAVBeats(int first_col, int first_line){
  //panLogo();
  osd.setPanel(first_col, first_line);
  osd.openPanel();
  osd.printf_P(PSTR("No MCU data!"));
  osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
// Size   : 1 x 5  (rows x chars)
// Staus  : done

void panGPSats(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if (osd_fix_type == 2)
      osd.printf("%c%c", 0x67, 0xAE);
    if (osd_fix_type == 3)
      osd.printf("%c%c", 0x67, 0xAD);
//    byte gps_str = 0x2a;
//    if (osd_fix_type == 2) gps_str = 0x1f;
//    if (osd_fix_type == 3) gps_str = 0x0f;
//    
//    if ((eph >= 200) && blinker)
//       gps_str = 0x20;
//    
//    osd.printf("%c%2i", gps_str, osd_satellites_visible);    
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols as on first char
// Size   : 2 x 12  (rows x chars)
// Staus  : done

void panGPS(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%10.6f|%c%10.6f", 0x03, (double)osd_lat, 0x04, (double)osd_lon);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
// Size   : 1 x 5  (rows x chars)
// Staus  : not ready

void panHeading(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4.0f%c", (double)osd_heading, 0x05);
    osd.closePanel();
}

/* **************************************************************** */
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
// Size   : 2 x 13  (rows x chars)
// Staus  : done

//void panRose(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    //osd_heading  = osd_yaw;
//    //if(osd_yaw < 0) osd_heading = 360 + osd_yaw;
////    osd.printf("%s|%c%s%c", "\x20\xc0\xc0\xc0\xc0\xc0\xc7\xc0\xc0\xc0\xc0\xc0\x20", 0xc3, buf_show, 0x87);
//    osd.printf("%c%s%c", 0xc3, buf_show, 0x87);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panWPDis
// Needs  : X, Y locations
// Output : W then distance in Km - Distance to next waypoint
// Size   : 1 x 2  (rows x chars)
// Staus  : not ready TODO - CHANGE the Waypoint symbol - Now only a W!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//void panWPDis(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    
////    wp_target_bearing_rotate_int = ((int)round(((float)wp_target_bearing - osd_heading)/360.0 * 16.0) + 16) % 16 + 1; //[1, 16]
//
//    if (wp_target_bearing > 0){
//      wp_target_bearing_rotate_int = round((wp_target_bearing - osd_heading)/360 *16.0) + 1; 
//    }else if (wp_target_bearing < 0){
//      wp_target_bearing_rotate_int = round(((360 + wp_target_bearing) - osd_heading)/360 *16.0) + 1;
//    } 
//    if (wp_target_bearing_rotate_int < 0) wp_target_bearing_rotate_int += 16;
//    if (wp_target_bearing_rotate_int == 0) wp_target_bearing_rotate_int = 16;
//    
//    if (xtrack_error > 999) xtrack_error = 999;
//    else if (xtrack_error < -999) xtrack_error = -999;
//
//    osd.printf("%c%c%2i%c%4.0f%c|",0x57, 0x70, wp_number,0x0,(double)((float)(wp_dist) * converth),high);
//    showArrow((uint8_t)wp_target_bearing_rotate_int,0);
//
//    if (osd_mode == 10){
//        osd.printf("%c%c%c%4.0f%c", 0x20, 0x58, 0x65, (xtrack_error* converth), high);
//    }else{
//        osd.printf_P(PSTR("\x20\x20\x20\x20\x20\x20\x20\x20"));
//    }
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
// Size   : 1 x 2  (rows x chars)
// Status : not tested

//void panHomeDir(int first_col, int first_line){
//    osd.setPanel(first_col, first_line);
//    osd.openPanel();
//    showArrow((uint8_t)osd_home_direction,0);
//    osd.closePanel();
//}

/* **************************************************************** */
// Panel  : panFlightMode 
// Needs  : X, Y locations
// Output : 2 symbols, one static name symbol and another that changes by flight modes
// Size   : 1 x 2  (rows x chars)
// Status : done

void panFlightMode(int first_col, int first_line){
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //char c1 = 0xE0 ;//"; char c2; char c3; char c4; char c5; 
    char* mode_str="";
    if (osd_mode == 0) mode_str = "stab"; //Stabilize
    else if (osd_mode == 1) mode_str = "acro"; //Acrobatic
    else if (osd_mode == 2) mode_str = "alth"; //Alt Hold
    else if (osd_mode == 3) mode_str = "auto"; //Auto
    else if (osd_mode == 4) mode_str = "guid"; //Guided
    else if (osd_mode == 5) mode_str = "loit"; //Loiter
    else if (osd_mode == 6) mode_str = "rtl "; //Return to Launch
    else if (osd_mode == 7) mode_str = "circ"; //Circle
    //else if (osd_mode == 8) mode_str = "posi"; //Position Hold (Old)
    else if (osd_mode == 9) mode_str = "land"; //Land
    else if (osd_mode == 10) mode_str = "oflo"; //OF_Loiter
    else if (osd_mode == 11) mode_str = "drif"; //Drift
    else if (osd_mode == 13) mode_str = "sprt"; //Sport
    //else if (osd_mode == 14) mode_str = "flip"; //Flip
    //else if (osd_mode == 15) mode_str = "tune"; //Tune
    else if (osd_mode == 16) mode_str = "phld"; //Position Hold (Earlier called Hybrid)
    osd.printf("%c%s%c", 0x7F, mode_str, motor_armed * 0x86);
    osd.closePanel();
}


// ---------------- EXTRA FUNCTIONS ----------------------
// Show those fancy 2 char arrows
//void showArrow(uint8_t rotate_arrow,uint8_t method) {  
//    int arrow_set1 = 0x90;
//    //We trust that we receive rotate_arrow [1, 16] so 
//    //it's no needed (rotate_arrow <= 16) in the if clause
//    arrow_set1 += rotate_arrow * 2 - 2;
//    //arrow_set2 = arrow_set1 + 1;
////    if(method == 1) osd.printf("%c%3.0f%c|%c%c%2.0f%c",0x1D,(double)(osd_windspeed * converts),spe, (byte)arrow_set1, (byte)(arrow_set1 + 1),(double)(osd_windspeedz * converts),spe);
//    if(method == 1) osd.printf("%c%3.0f%c|%c%c%2.0f%c",0x1d,(double)(osd_windspeed * converts),spe, arrow_set1, arrow_set1 + 1,(double)(nor_osd_windspeed * converts),spe);
//    else if(method == 2) osd.printf("%c%c%4i%c", arrow_set1, arrow_set1 + 1, off_course, 0x05);   
//    else osd.printf("%c%c", arrow_set1, arrow_set1 + 1);
//}

// Calculate and shows Artificial Horizon
// Smooth horizon by Jörg Rothfuchs
							// with different factors we can adapt do different cam optics
#define AH_PITCH_FACTOR		0.010471976		// conversion factor for pitch
#define AH_ROLL_FACTOR		0.017453293		// conversion factor for roll
//#define AH_COLS			12			// number of artificial horizon columns
//#define AH_ROWS			11			// number of artificial horizon rows
#define CHAR_COLS		12			// number of MAX7456 char columns
#define CHAR_ROWS		18			// number of MAX7456 char rows
#define CHAR_SPECIAL		9			// number of MAX7456 special chars for the artificial horizon
#define AH_TOTAL_LINES		AH_ROWS * CHAR_ROWS	// helper define


#define LINE_SET_STRAIGHT__	(0xC7 - 1)		// code of the first MAX7456 straight char -1
#define LINE_SET_STRAIGHT_O	(0xD0 - 3)		// code of the first MAX7456 straight overflow char -3
#define LINE_SET_P___STAG_1	(0xD1 - 1)		// code of the first MAX7456 positive staggered set 1 char -1
#define LINE_SET_P___STAG_2	(0xDA - 1)		// code of the first MAX7456 positive staggered set 2 char -1
#define LINE_SET_N___STAG_1	(0xE3 - 1)		// code of the first MAX7456 negative staggered set 1 char -1
#define LINE_SET_N___STAG_2	(0xEC - 1)		// code of the first MAX7456 negative staggered set 2 char -1
#define LINE_SET_P_O_STAG_1	(0xF5 - 2)		// code of the first MAX7456 positive overflow staggered set 1 char -2
#define LINE_SET_P_O_STAG_2	(0xF9 - 1)		// code of the first MAX7456 positive overflow staggered set 2 char -1
#define LINE_SET_N_O_STAG_1	(0xF7 - 2)		// code of the first MAX7456 negative overflow staggered set 1 char -2
#define LINE_SET_N_O_STAG_2	(0xFC - 1)		// code of the first MAX7456 negative overflow staggered set 2 char -1


#define OVERFLOW_CHAR_OFFSET	6			// offset for the overflow subvals


#define ANGLE_1			9			// angle above we switch to line set 1
#define ANGLE_2			25			// angle above we switch to line set 2


// Calculate and show artificial horizon
// used formula: y = m * x + n <=> y = tan(a) * x + n
void showHorizon(int start_col, int start_row) {
    int col, row, pitch_line, middle, hit, subval;
    int roll;
    int line_set = LINE_SET_STRAIGHT__;
    int line_set_overflow = LINE_SET_STRAIGHT_O;
    int subval_overflow = 9;
    
    // preset the line char attributes
    roll = osd_roll;
    if ((roll >= 0 && roll < 90) || (roll >= -179 && roll < -90)) {	// positive angle line chars
	roll = roll < 0 ? roll + 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_P___STAG_2;
	    line_set_overflow = LINE_SET_P_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_P___STAG_1;
	    line_set_overflow = LINE_SET_P_O_STAG_1;
            subval_overflow = 8;
	}
    } else {								// negative angle line chars
	roll = roll > 90 ? roll - 179 : roll;
        if (abs(roll) > ANGLE_2) {
	    line_set = LINE_SET_N___STAG_2;
	    line_set_overflow = LINE_SET_N_O_STAG_2;
            subval_overflow = 7;
	} else if (abs(roll) > ANGLE_1) {
	    line_set = LINE_SET_N___STAG_1;
	    line_set_overflow = LINE_SET_N_O_STAG_1;
            subval_overflow = 8;
	}
    }
    
    pitch_line = round(tan(-AH_PITCH_FACTOR * osd_pitch) * AH_TOTAL_LINES) + AH_TOTAL_LINES/2;	// 90 total lines
    for (col=1; col<=AH_COLS; col++) {
        middle = col * CHAR_COLS - (AH_COLS/2 * CHAR_COLS) - CHAR_COLS/2;	  // -66 to +66	center X point at middle of each column
        hit = tan(AH_ROLL_FACTOR * osd_roll) * middle + pitch_line;	          // 1 to 90	calculating hit point on Y plus offset
        if (hit >= 1 && hit <= AH_TOTAL_LINES) {
	    row = (hit-1) / CHAR_ROWS;						  // 0 to 4 bottom-up
	    subval = (hit - (row * CHAR_ROWS) + 1) / (CHAR_ROWS / CHAR_SPECIAL);  // 1 to 9
	    
	    // print the line char
            osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 1);
            osd.printf("%c", line_set + subval);
	    
	    // check if we have to print an overflow line char
	    if (subval >= subval_overflow && row < 4) {	// only if it is a char which needs overflow and if it is not the upper most row
                osd.openSingle(start_col + col - 1, start_row + AH_ROWS - row - 2);
                osd.printf("%c", line_set_overflow + subval - OVERFLOW_CHAR_OFFSET);
	    }
        }
    }
}

// Calculate and shows verical speed aid
//void showILS(int start_col, int start_row) { 
//    //Show line on panel center because horizon line can be
//    //high or low depending on pitch attitude
//    int subval_char = 0xCF;
//
//    //shift alt interval from [-5, 5] to [0, 10] interval, so we
//    //can work with remainders.
//    //We are using a 0.2 altitude units as resolution (1 decimal place)
//    //so convert we convert it to times 10 to work 
//    //only with integers and save some bytes
//    //int alt = (osd_alt_to_home * converth + 5) * 10;
//    int alt = (osd_alt_rel * converth + 5) * 4.4; //44 possible position 5 rows times 9 chars
//    
//    if((alt < 44) && (alt > 0)){
//        //We have 9 possible chars
//        //(alt * 5) -> 5 represents 1/5 which is our resolution. Every single
//        //line (char) change represents 0,2 altitude units
//        //% 10 -> Represents our 10 possible characters
//        //9 - -> Inverts our selected char because when we gain altitude
//        //the selected char has a lower position in memory
//        //+ 5 -> Is the memory displacement od the first altitude charecter 
//        //in memory (it starts at 0x05
//        //subval_char = (99 - ((alt * 5) % 100)) / 9 + 0xC7;
//        subval_char = (8 - (alt  % 9)) + 0xC7;
//        //Each row represents 2 altitude units
//        start_row += (alt / 9);
//    }
//    else if(alt >= 44){
//        //Copter is too high. Ground is way too low to show on panel, 
//        //so show down arrow at the bottom
//        subval_char = 0xC8; 
//        start_row += 4;
//    }
//
//    //Enough calculations. Let's show the result
//    osd.openSingle(start_col + AH_COLS + 2, start_row);
//    osd.printf("%c", subval_char);
//}

void do_converts()
{
    if (EEPROM.read(measure_ADDR) == 0) {
        converts = 3.6;
        converth = 1.0;
        spe = 0x10;
        high = 0x0c;
        temps = 0xba;
        tempconv = 10;
        tempconvAdd = 0;
        distchar = 0x1b;
        distconv = 1000;
        climbchar = 0x1a;
    } else {
        converts = 2.23;
        converth = 3.28;
        spe = 0x19;
        high = 0x66;
        temps = 0xbb;
        tempconv = 18;
        tempconvAdd = 3200;
        distchar = 0x1c;
        distconv = 5280;
        climbchar = 0x1e;
    }
}

void timers()
{
  if (one_sec_timer_switch == 1){ 
    one_sec_timer = millis() + 500;
    one_sec_timer_switch = 0;
    blinker = !blinker;
  }
  if (millis() > one_sec_timer) one_sec_timer_switch = 1;  
}
