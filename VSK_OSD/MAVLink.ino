#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

uint8_t rc_port;
uint8_t sv_port;

void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);
    }
}

void read_mavlink(){
    mavlink_message_t msg; 
    mavlink_status_t status;

    //grabing data 
    while(Serial.available() > 0) { 
        uint8_t c = Serial.read();

        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                uploadFont();
            }
        }

        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            lastMAVBeat = millis();
            mavlink_active = 1;
            //handle msg
//            Serial.printf("%i ", msg.msgid);
            switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavbeat = 1;
                    apm_mav_system    = msg.sysid;
                    apm_mav_component = msg.compid;
                 //   apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //Mode (arducoper armed/disarmed)
                    base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                    //if(getBit(base_mode,7)) motor_armed = 1;
                    //else motor_armed = 0;
                    motor_armed = getBit(base_mode,7);

                    osd_nav_mode = 0;          
                    /*lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }*/
                }
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                {

                    osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                    osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)         
                    osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
                    //osd_mode = apm_mav_component;//Debug
                    //osd_nav_mode = apm_mav_system;//Debug
//                    Serial.print(osd_curr_A);
//                    Serial.print(osd_vbat_A);
//                    Serial.print(osd_battery_remaining_A);
//                    if(osd_vbat_A > 14.0f){
//                       batt_warn_level = f; 
//                    } else if (osd_vbat_A >10.5f){
                       batt_warn_level = 10.5f; 
//                    } else {
//                       batt_warn_level = 7.6f;
//                    }
                }
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    osd_alt_gps = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000.0f;
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                    osd_cog = mavlink_msg_gps_raw_int_get_cog(&msg);
                    eph = mavlink_msg_gps_raw_int_get_eph(&msg);
                }
                break; 
            case MAVLINK_MSG_ID_VFR_HUD:
                {
                    osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                    osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                    osd_throttle = (uint8_t)mavlink_msg_vfr_hud_get_throttle(&msg);
                    osd_alt_rel = mavlink_msg_vfr_hud_get_alt(&msg);
                    osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                }
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                {
                    osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
//                    Serial.printf("%4.0i%c", osd_pitch, ' ');
                    osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
//                    Serial.printf("Roll:%6.2i ", osd_roll);
                    osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
                }
                break;
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                {
//                  nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
//                  nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
//                  nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
                  wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
                  wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
//                  alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
//                  aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
                  xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_CURRENT:
                {
                    wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
                }
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
                    rc_port = mavlink_msg_rc_channels_raw_get_port(&msg);
                    if (rc_port == 0){
                      chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
                      chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
                      chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
                      chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
                      chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                      chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                      chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                      chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                      osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
                    }
                }
                break;          
//            case MAVLINK_MSG_ID_WIND:
//                {
//                    osd_winddirection = mavlink_msg_wind_get_direction(&msg); // 0..360 deg, 0=north
//                    osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
////                    osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
//                }
//                break;
            case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
                }
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 
                { 
                    //osd_home_alt = osd_alt - (mavlink_msg_global_position_int_get_relative_alt(&msg)*0.001); 
                    //Commented because it seems that we only get relative alt when we have GPS lock.
                    //That shouldn't be because we may rely only on baro. So using vfr hud alt (testing)
                    //osd_alt_rel = (mavlink_msg_global_position_int_get_relative_alt(&msg)*0.001);
                }
                break;
            case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    rssi2 = mavlink_msg_radio_status_get_rssi(&msg);
//                    Serial.printf("Radio status %d ", rssi2);
                }
                break;
            case MAVLINK_MSG_ID_VSKYLINE_OSD_STATUS:
                {
//                    page_id = mavlink_msg_vskyline_osd_status_get_osd_status(&msg);
//                    Serial.printf_P(PSTR("osd status %x "), page_id);  
                      page_id = 0x0100;                   
                        //menu page
                    if ((page_id & 0xff00) == 0x0100){
                       panel = 4;
                       subpage = 0;
                       pos_line = (int8_t)(page_id & 0x000f) + 4;
                       pos_col = 3;
                    }
                    //rc setup
                    else if ((page_id &0xfff0) == 0x0200){
                       panel = 4;
                       subpage = 1;
                       pos_line = (int8_t)(page_id & 0x000f);
                    }
                    //rc calib
                    else if ((page_id &0xfff0) == 0x0210){
                       panel = 4;
                       subpage = 7;
                    }
                    //radio status
                    else if ((page_id &0xfff0) == 0x0220){
                       panel = 4;
                       subpage = 8;
                    }
                    //PID setup
                    else if ((page_id & 0xff00) == 0x0300){
                       panel = 4;
                       subpage = 5;
                       pos_line = (int8_t)(page_id & 0x000f);
                    }
                    //IMU setup
                    else if ((page_id & 0xff00) == 0x0400){
                       panel = 4;
                       subpage = 3;
                       pos_line = (int8_t)(page_id & 0x000f);
                    }
                    //motor setup
                    else if ((page_id & 0xff00) == 0x0500){
                       panel = 4;
                       subpage = 4;
                       pos_line = (int8_t)(page_id & 0x000f);
                    }
                    //PID page
                    else if((page_id & 0xfff0) == 0x0600){
                       panel = 4;
                       subpage = 2;
                       pos_line = (int8_t)(page_id & 0x000f);
                    }
                    else if((page_id &0xfff0) == 0x0700){
                       panel = 4;
                       subpage = 9;
                    }
                    //main flying page
                    else if ((page_id &0xff00) == 0x1100){
                       panel = 0;
                    }
                    //lite 1
                    else if ((page_id &0xff00) == 0x1200){
                       panel = 1;
                    }
                    //lite 2
                    else if ((page_id &0xff00) == 0x1300){
                       panel = 2;
                    }
                    //pid page
                    else if ((page_id &0xff00) == 0x2100){
                       panel = 3;
                    }
                    else {
                      panel = 0;
//                        subpage = 0;
//                        pos_line = (int8_t)(page_id & 0x000f) + 3;
//                        pos_col = 3;
                    }

//                    Serial.printf("Radio status %d ", rssi2);
                    name32 = mavlink_msg_vskyline_osd_status_get_pilot_name(&msg);
//                    uint8_t video_pal = mavlink_msg_vskyline_osd_status_get_video_pal(&msg);
                    uint8_t video_pal = 0;
                    if(video_pal != video_mode_old){
                      EEPROM.write(PAL_NTSC_ADDR, video_pal);
                      osd.init();
                      video_mode_old = video_pal;
//                      Serial.println("Mode change");
//                      Serial.println(video_pal);
                    }
                    
//                    Serial.printf("Pilot name %x %i ", name32, name32);
                }
                break;
            case MAVLINK_MSG_ID_VSKYLINE_PID_SETUP:
                {                   
//                    Serial.print("VSkyline PID");
                    profile = mavlink_msg_vskyline_pid_setup_get_profile(&msg);
                    pidtype = mavlink_msg_vskyline_pid_setup_get_pid_type(&msg);
                    p_pitch = mavlink_msg_vskyline_pid_setup_get_p_pitch(&msg);
                    i_pitch = mavlink_msg_vskyline_pid_setup_get_i_pitch(&msg);
                    d_pitch = mavlink_msg_vskyline_pid_setup_get_d_pitch(&msg);
                    rate_pitch = mavlink_msg_vskyline_pid_setup_get_rate_pitch(&msg);
                    p_roll = mavlink_msg_vskyline_pid_setup_get_p_roll(&msg);
                    i_roll = mavlink_msg_vskyline_pid_setup_get_i_roll(&msg);
                    d_roll = mavlink_msg_vskyline_pid_setup_get_d_roll(&msg);
                    rate_roll = mavlink_msg_vskyline_pid_setup_get_rate_roll(&msg);
                    p_yaw = mavlink_msg_vskyline_pid_setup_get_p_yaw(&msg);
                    i_yaw = mavlink_msg_vskyline_pid_setup_get_i_yaw(&msg);
                    d_yaw = mavlink_msg_vskyline_pid_setup_get_d_yaw(&msg);
                    rate_yaw = mavlink_msg_vskyline_pid_setup_get_rate_yaw(&msg);                    
                }
                break;
            default:
                //Do nothing
                break;
            }
        }
        delayMicroseconds(138);
        //next one
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}
