  #include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
  #include "../GCS_MAVLink/include/mavlink/v1.0/common/mavlink.h"
  static bool mavlink_active;
  static uint8_t crlf_count = 0;
  static int packet_drops = 0;
  static int parse_error = 0;
  
  
  
  // Replcae Mavlink with own comm 
  
 void read_mavlink(){
    
   float lat_1;
   float lat_2;
   float lon_1;
   float lon_2;
   
    while(Serial.available() >= 8) {
      
      if(loop_counter == 0)
      {
      c1 = Serial.read();
      c2 = Serial.read();
      }
      else
      {
        c1 = c2;
        c2 = Serial.read();
      }
      
      if(c1==B10000001 && c2==B10000001)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            osd_pitch = (((d1 << 8)+d2)/80-90);
            d1 = Serial.read();
            d2 = Serial.read();
            osd_roll = (((d1 << 8)+d2)/40-180);
            d1 = Serial.read();
            d2 = Serial.read();
            osd_heading = (((d1 << 8)+d2)/40);
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
      else if(c1==B10000010 && c2==B10000010)
      {
        osd_dt_step = Serial.read();
        motor_armed = Serial.read();
        d1= Serial.read();
        d2 = Serial.read();
        osd_throttle = (((d1 << 8)+d2)/150);
        d1 = Serial.read();
        d2 = Serial.read();
        osd_nav_mode = (((d1 << 8)+d2));
            
        lastMAVBeat = millis();
        loop_counter = 0;
      }
      else if(c1==B10000011 && c2==B10000011)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            osd_alt = (((d1 << 8)+d2));
            d1 = Serial.read();
            d2 = Serial.read();
            osd_climb = (((d1 << 8)+d2)/100);
            d1 = Serial.read();
            d2 = Serial.read();
            osd_ultra_alt = (((d1 << 8)+d2)/200);
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
      else if(c1==B10000100 && c2==B10000100)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            osd_rssi = (((d1 << 8)+d2));
            d1 = Serial.read();
            d2 = Serial.read();
            osd_vbat_A = ((float)((d1 << 8)+d2))/100;
            d1 = Serial.read();
            d2 = Serial.read();
            osd_curr_A = ((float)((d1 << 8)+d2))/100;
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
       else if(c1==B10000101 && c2==B10000101)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            osd_home_distance = (long)(((d1 << 8)+d2));
            d1 = Serial.read();
            d2 = Serial.read();
            osd_home_direction = round((float)(((float)((d1 << 8)+d2))/40/360.0f) * 16.0f) + 1;
            d1 = Serial.read();
            d2 = Serial.read();
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
       else if(c1==B10000110 && c2==B10000110)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            lat_1 = ((float)((d1 << 8)+d2))/100;
            d1 = Serial.read();
            d2 = Serial.read();
            lat_2 = ((float)((d1 << 8)+d2))/1000/100;
            osd_lat = lat_1+lat_2;         
           d1 = Serial.read();
            d2 = Serial.read();
           osd_fix_type = ((d1 << 8)+d2);
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
           else if(c1==B10000111 && c2==B10000111)
      {
            d1 = Serial.read();
            d2 = Serial.read();
            lon_1 = ((float)((d1 << 8)+d2))/100;
            d1 = Serial.read();
            d2 = Serial.read();
            lon_2 = ((float)((d1 << 8)+d2))/1000/100;
            osd_lon = lon_1+lon_2;         
            d1 = Serial.read();
            d2 = Serial.read();
            osd_satellites_visible = ((d1 << 8)+d2);
           
            lastMAVBeat = millis();
            loop_counter = 0;
      }
      else
      {
        loop_counter = 1; 
      }
      
    }
      
    /*
      mavlink_message_t msg; 
      mavlink_status_t status;
  
      //grabing data 
      while(Serial.available() > 0) { 
          uint8_t c = Serial.read();
  
         // allow CLI to be started by hitting enter 3 times, if no
          //heartbeat packets have been received
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
              mavlink_active = 1;
              //handle msg
              switch(msg.msgid) {
              case MAVLINK_MSG_ID_HEARTBEAT:
                  {
                      mavbeat = 1;
                      apm_mav_system    = msg.sysid;
                      apm_mav_component = msg.compid;
                      apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                   //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                      osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                      //Mode (arducoper armed/disarmed)
                      base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                      if(getBit(base_mode,7)) motor_armed = 1;
                      else motor_armed = 0;
  
                      osd_nav_mode = 0;          
                      lastMAVBeat = millis();
                      if(waitingMAVBeats == 1){
                          enable_mav_request = 1;
                      }
                  }
                  break;
              case MAVLINK_MSG_ID_SYS_STATUS:
                  {
  
                      osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                      osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)         
                      osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
                      //osd_mode = apm_mav_component;//Debug
                      //osd_nav_mode = apm_mav_system;//Debug
                  }
                  break;
  
              case MAVLINK_MSG_ID_GPS_RAW_INT:
                  {
                      osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                      osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                      osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                      osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                  }
                  break; 
              case MAVLINK_MSG_ID_VFR_HUD:
                  {
                      osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                      osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                      osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                      osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                      //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
                      //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
                      osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                      osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                  }
                  break;
              case MAVLINK_MSG_ID_ATTITUDE:
                  {
                      osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
                      osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
                      osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
                  }
                  break;
              case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                  {
                    nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
                    nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
                    nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
                    wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
                    wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
                    alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
                    aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
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
                      chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
                      chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
                      osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                      osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                      osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                      osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                      osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
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
      */
      //packet_drops += status.packet_rx_drop_count;
     // parse_error += status.parse_error;
  
  }
  
  void request_mavlink_rates()
  {
    /*
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
      */
  }
  

